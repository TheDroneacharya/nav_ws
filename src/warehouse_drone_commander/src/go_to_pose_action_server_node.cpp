#include <chrono>
#include <functional>
#include <memory>
#include <thread>
#include <cmath>
#include <mutex>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <mavros_msgs/msg/position_target.hpp>

#include <warehouse_drone_interfaces/action/go_to_pose.hpp>

using namespace std::placeholders;
using namespace std::chrono_literals;

class GoToPoseActionServer : public rclcpp::Node
{
public:
    GoToPoseActionServer() : Node("go_to_pose_action_server")
    {
        // Initialization code here
        RCLCPP_INFO(this->get_logger(), "Starting Go To Pose Action Server node...");

        // TODO: parse parameters from yaml file
        this->declare_parameter<double>("goal_tolerance_xy", 0.1);          // xy tolerance radius in meters
        this->declare_parameter<double>("goal_tolerance_yaw", 0.1);         // yaw tolerance in radians (currently unused)
        this->declare_parameter<double>("goal_tolerance_height", 0.1);      // height tolerance in meters
        this->declare_parameter<double>("goal_confirmation_delay", 1.0);    // Amount of time in seconds to observe before confirming that the goal has been achieved

        goal_tolerance_xy_ = this->get_parameter("goal_tolerance_xy").as_double();
        goal_tolerance_yaw_ = this->get_parameter("goal_tolerance_yaw").as_double();
        goal_tolerance_height_ = this->get_parameter("goal_tolerance_height").as_double();
        goal_confirmation_delay_ = this->get_parameter("goal_confirmation_delay").as_double();

        auto sensor_qos = rclcpp::SensorDataQoS();

        // Subscribers and Publishers
        local_position_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>("mavros/local_position/pose", sensor_qos, std::bind(&GoToPoseActionServer::local_position_callback, this, _1));
        setpoint_pub_ = this->create_publisher<mavros_msgs::msg::PositionTarget>("go_to_pose_action_server/setpoint_raw/local", sensor_qos);

        // Action Server
        action_server_ = rclcpp_action::create_server<warehouse_drone_interfaces::action::GoToPose>(
            this,
            "go_to_pose",
            std::bind(&GoToPoseActionServer::handle_goal, this, _1, _2),
            std::bind(&GoToPoseActionServer::handle_cancel, this, _1),
            std::bind(&GoToPoseActionServer::handle_accepted, this, _1));
    }

private:

    ///////////////////////////////////// Subscription Callbacks /////////////////////////////////////

    void local_position_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg){
        std::lock_guard<std::mutex> lock(pose_mutex_);
        current_pose_ = *msg;
        if (!pose_received_) {
            RCLCPP_INFO(this->get_logger(), "First local pose received by Go To Pose Action Server.");
            pose_received_ = true;
        }
    }

    ///////////////////////////////////// Action Server Callbacks /////////////////////////////////////

    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID & uuid,
        [[maybe_unused]] std::shared_ptr<const warehouse_drone_interfaces::action::GoToPose::Goal> goal)
    {
        if (!pose_received_) {
            RCLCPP_WARN(this->get_logger(), "Cannot accept goal: waiting for initial pose message...");
            return rclcpp_action::GoalResponse::REJECT;
        }
        RCLCPP_WARN(this->get_logger(), "Go To Pose Action Server received goal request with target pose: (%.2f, %.2f, %.2f)",
                    goal->goal_pose.pose.position.x,
                    goal->goal_pose.pose.position.y,
                    goal->goal_pose.pose.position.z);
        (void)uuid;
        // Accept all goals
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<warehouse_drone_interfaces::action::GoToPose>> goal_handle)
    {
        RCLCPP_WARN(this->get_logger(), "Go To Pose Action Server received request to cancel goal pose: (%.2f, %.2f, %.2f)",
                    goal_handle->get_goal()->goal_pose.pose.position.x,
                    goal_handle->get_goal()->goal_pose.pose.position.y,
                    goal_handle->get_goal()->goal_pose.pose.position.z);
        (void)goal_handle;
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<warehouse_drone_interfaces::action::GoToPose>> goal_handle)
    {
        // This needs to return quickly to avoid blocking the executor, so spin up a new thread
        std::thread{std::bind(&GoToPoseActionServer::execute, this, _1), goal_handle}.detach();
    }

    ////////////////////////////////////// Core Functions /////////////////////////////////////

    void execute(const std::shared_ptr<rclcpp_action::ServerGoalHandle<warehouse_drone_interfaces::action::GoToPose>> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Go To Pose Action Server executing goal pose: (%.2f, %.2f, %.2f)",
                    goal_handle->get_goal()->goal_pose.pose.position.x,
                    goal_handle->get_goal()->goal_pose.pose.position.y,
                    goal_handle->get_goal()->goal_pose.pose.position.z);

        const auto goal = goal_handle->get_goal();
        auto feedback = std::make_shared<warehouse_drone_interfaces::action::GoToPose::Feedback>();
        auto result = std::make_shared<warehouse_drone_interfaces::action::GoToPose::Result>();

        auto start_time = this->now();

        bool goal_area_entered = false;
        auto goal_entered_time = this->now();

        // Main loop to move to goal pose
        rclcpp::Rate rate(1); // 1 Hz
        while (rclcpp::ok()) {
            // Check if goal is canceled
            if (goal_handle->is_canceling()) {
                RCLCPP_WARN(this->get_logger(), "Goal canceled.");
                result->success = false;
                goal_handle->canceled(result);
                return;
            }

            // TODO: check if correct
            // Lock pose for reading
            geometry_msgs::msg::PoseStamped current_pose_copy;
            {
                std::lock_guard<std::mutex> lock(pose_mutex_);
                current_pose_copy = current_pose_;
            }

            // Update setpoint to goal pose
            update_setpoint(
                std::make_shared<geometry_msgs::msg::PoseStamped>(goal->goal_pose),
                goal->max_velocity_x,
                goal->max_velocity_y,
                goal->max_velocity_z,
                goal->max_yaw_rate);
            publish_setpoint();

            // Compute distance to goal pose
            double dx = std::fabs(current_pose_copy.pose.position.x - goal->goal_pose.pose.position.x);
            double dy = std::fabs(current_pose_copy.pose.position.y - goal->goal_pose.pose.position.y);
            double dz = std::fabs(current_pose_copy.pose.position.z - goal->goal_pose.pose.position.z);
            double distance_to_goal = std::sqrt(dx*dx + dy*dy + dz*dz);
            double dxy = std::sqrt(dx*dx + dy*dy);
            double dyaw = std::fabs(quaternion_to_yaw(current_pose_copy.pose.orientation) - quaternion_to_yaw(goal->goal_pose.pose.orientation));
            if (dyaw > M_PI) {
                dyaw = 2 * M_PI - dyaw; // Normalize yaw difference to [0, pi]
            }

            // Compute elapsed time
            double elapsed_time = (this->now() - start_time).seconds();

            // Provide feedback
            feedback->distance_remaining = distance_to_goal;
            feedback->elapsed_time = elapsed_time;
            feedback->goal_pose = goal->goal_pose;
            goal_handle->publish_feedback(feedback);
            RCLCPP_INFO(this->get_logger(), "Current distance to target: %.2f m", distance_to_goal);

            bool within_xy_tolerance = (dxy <= goal_tolerance_xy_);
            bool within_height_tolerance = (dz <= goal_tolerance_height_);
            bool within_yaw_tolerance = (dyaw <= goal_tolerance_yaw_);
            bool within_all_tolerances = within_xy_tolerance && within_height_tolerance && within_yaw_tolerance;

            // Check if within tolerance
            if (within_all_tolerances && !goal_area_entered) {
                goal_area_entered = true;
                goal_entered_time = this->now();
                RCLCPP_WARN(this->get_logger(), "Goal tolerance area entered, waiting 1s to confirm...");
            } else if (within_all_tolerances && goal_area_entered) {
                // Check if within tolerance for required duration
                auto now = this->now();
                if ((now - goal_entered_time).seconds() >= goal_confirmation_delay_) {
                    RCLCPP_WARN(this->get_logger(), "Goal reached and confirmed.");
                    result->success = true;
                    result->elapsed_time = elapsed_time;
                    goal_handle->succeed(result);
                    return;
                }
            } else {
                // Reset if moved out of tolerance
                if (goal_area_entered) {
                    RCLCPP_WARN(this->get_logger(), "Moved out of goal tolerance area, resetting confirmation timer.");
                }
                goal_area_entered = false;

                RCLCPP_INFO(this->get_logger(), "Not yet within goal tolerance: dxy=%.2f m (tolerance=%.2f m), dz=%.2f m (tolerance=%.2f m), dyaw=%.2f rad (tolerance=%.2f rad).", 
                            dxy, goal_tolerance_xy_,
                            dz, goal_tolerance_height_,
                            dyaw, goal_tolerance_yaw_);
            }

            rate.sleep();
        }

        // If we exit the loop without reaching the goal
        RCLCPP_WARN(this->get_logger(), "Goal execution aborted.");
        result->success = false;
        goal_handle->abort(result);
    }

    ///////////////////////////////////// Utility Functions /////////////////////////////////////

    void update_setpoint(
        const geometry_msgs::msg::PoseStamped::SharedPtr pose = nullptr,
        const double max_velocity_x = 0,
        const double max_velocity_y = 0,
        const double max_velocity_z = 0,
        const double max_yaw_rate = 0){

        std::lock_guard<std::mutex> lock(setpoint_mutex_);
        // Update the setpoint_to_send_ message with desired values
        // If a message is provided, use its values
        if (pose != nullptr) {
            // TODO: Check values
            setpoint_to_send_.header.frame_id = "base_link";    // TODO: paramterize this from yaml file
            setpoint_to_send_.coordinate_frame = mavros_msgs::msg::PositionTarget::FRAME_LOCAL_NED;
            setpoint_to_send_.type_mask =   mavros_msgs::msg::PositionTarget::IGNORE_AFX |
                                            mavros_msgs::msg::PositionTarget::IGNORE_AFY |
                                            mavros_msgs::msg::PositionTarget::IGNORE_AFZ;  // Use position, velocity, yaw, yaw_rate
            setpoint_to_send_.position.x = pose->pose.position.x;
            setpoint_to_send_.position.y = pose->pose.position.y;
            setpoint_to_send_.position.z = pose->pose.position.z;
            setpoint_to_send_.velocity.x = max_velocity_x;
            setpoint_to_send_.velocity.y = max_velocity_y;
            setpoint_to_send_.velocity.z = max_velocity_z;
            setpoint_to_send_.acceleration_or_force.x = 0;
            setpoint_to_send_.acceleration_or_force.y = 0;
            setpoint_to_send_.acceleration_or_force.z = 0;
            setpoint_to_send_.yaw = quaternion_to_yaw(pose->pose.orientation);
            setpoint_to_send_.yaw_rate = max_yaw_rate;
        } else {
            RCLCPP_WARN(this->get_logger(), "update_setpoint() called without argument... No new pose provided, maintaining previous setpoint.");
        }
    }

    void publish_setpoint(){
        std::lock_guard<std::mutex> lock(setpoint_mutex_);
        // Set appropriate header timestamp
        setpoint_to_send_.header.stamp = this->now();
        // Publish the current setpoint
        setpoint_pub_->publish(setpoint_to_send_);
    }

    // Utility function to extract yaw from quaternion (output in radians)
    double quaternion_to_yaw(const geometry_msgs::msg::Quaternion &q) {
        // Yaw (z-axis rotation) from quaternion
        double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
        double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
        return std::atan2(siny_cosp, cosy_cosp);
    }

    ///////////////////////////////////// Parameters /////////////////////////////////////

    double goal_tolerance_xy_;
    double goal_tolerance_yaw_;
    double goal_tolerance_height_;
    double goal_confirmation_delay_;

    /////////////////////////////////////// ROS Interfaces /////////////////////////////////////

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr local_position_sub_;
    rclcpp::Publisher<mavros_msgs::msg::PositionTarget>::SharedPtr setpoint_pub_;

    rclcpp_action::Server<warehouse_drone_interfaces::action::GoToPose>::SharedPtr action_server_;

    ///////////////////////////////////// State Variables /////////////////////////////////////

    geometry_msgs::msg::PoseStamped current_pose_;
    mavros_msgs::msg::PositionTarget setpoint_to_send_;

    /////////////////////////////////////// Flags /////////////////////////////////////

    bool pose_received_{false};

    /////////////////////////////////// Mutexes /////////////////////////////////////

    std::mutex pose_mutex_; // Protects current_pose_ and pose_received_
    std::mutex setpoint_mutex_; // Protects setpoint_to_send_
};

int main(int argc, char *argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<GoToPoseActionServer>());
	rclcpp::shutdown();
	return 0;
}
