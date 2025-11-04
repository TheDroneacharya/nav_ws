#include <chrono>
#include <functional>
#include <memory>
#include <cmath>
#include <atomic>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp/timer.hpp>
#include <mutex>
#include <mavros_msgs/msg/rc_in.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <mavros_msgs/msg/state.hpp>
#include <mavros_msgs/msg/position_target.hpp>
#include <mavros_msgs/srv/set_mode.hpp>
#include <mavros_msgs/srv/command_bool.hpp>

#include <warehouse_drone_commander/action/go_to_pose.hpp>

using namespace std::placeholders;
using namespace std::chrono_literals;

class OffboardCommander : public rclcpp::Node
{
public:
    explicit OffboardCommander(const rclcpp::NodeOptions& options)
    : Node("offboard_commander", options)
    {
        // Initialization code here
        RCLCPP_INFO(this->get_logger(), "Starting offboard commander node...");

        // TODO: parse parameters from yaml file
        takeoff_height_ = 1.0f;  // meters
        max_velocity_x = 1.0f;   // m/s
        max_velocity_y = 1.0f;   // m/s
        max_velocity_z = 1.0f;   // m/s
        max_yaw_rate = 0.5f;     // rad/s
        go_to_pose_timeout_ = 60.0f; // seconds

        // Callback groups
        auto commander_callback_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        auto action_callback_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

        rc_sub_ = this->create_subscription<mavros_msgs::msg::RCIn>("/mavros/rc/in", 10, std::bind(&OffboardCommander::rc_in_callback, this, _1))
        state_sub_ = this->create_subscription<mavros_msgs::msg::State>("mavros/state", 10, std::bind(&OffboardCommander::state_callback, this, _1));
        local_position_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>("mavros/local_position/pose", 10, std::bind(&OffboardCommander::local_position_callback, this, _1));
        waypoint_path_sub_ = this->create_subscription<nav_msgs::msg::Path>("waypoint_generator/path", 10, std::bind(&OffboardCommander::waypoint_path_callback, this, _1));
        setpoint_pub_ = this->create_publisher<mavros_msgs::msg::PositionTarget>("offboard_commander/setpoint_raw/local", 10);
        arming_client_ = this->create_client<mavros_msgs::srv::CommandBool>("mavros/cmd/arming");
        set_mode_client_ = this->create_client<mavros_msgs::srv::SetMode>("mavros/set_mode");

        // Timers
        loop_timer_ = this->create_wall_timer(500ms, std::bind(&OffboardCommander::loop_timer_callback, this), commander_callback_group);  // 500ms = 2 Hz timer, callback group used to avoid service callbacks being in the same callback group as calling code

        // Action client
        go_to_pose_action_client_ = rclcpp_action::create_client<warehouse_drone_commander::action::GoToPose>(this, "go_to_pose", action_callback_group);
    }

private:

    ///////////////////////////////////// Callbacks /////////////////////////////////////

    void state_callback(const mavros_msgs::msg::State::SharedPtr msg){
        {
            std::lock_guard<std::mutex> lock(current_state_mutex_);
            current_state_ = *msg;
        }
        if (!state_received_.load()) {
            RCLCPP_INFO(this->get_logger(), "First MAVROS state received.");
            state_received_.store(true);
        }
    }

    void rc_in_callback(const mavros_msgs::msg::RCIn::SharedPtr msg){
        // Process RC input to set flags for offboard mode, arming, takeoff, etc.
        // RC channels are typically 1-indexed on transmitters, but 0-indexed in arrays.
        // So, channel 5 would be index 4.
        // TODO: Adjust channel indices and thresholds based on specific RC transmitter configuration
        const int ARMING_CHANNEL_INDEX = 9; // Assuming channel 10 is the arming switch
        const int OFFBOARD_CHANNEL_INDEX = 4; // Assuming channel 5 is the offboard switch
        const int TOTAL_CHANNELS = 12; // Assuming 12 channels total
        const int ARM_CHANNEL_THRESHOLD = 1500; // Threshold value for arming switch
        const int OFFBOARD_CHANNEL_THRESHOLD = 1500; // Threshold value for offboard switch

        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500, "RC Channels received: No. of channels: %zu", msg->channels.size());
        if (msg->channels.size() == TOTAL_CHANNELS) {
            // If rc was previously disconnected, log connection established
            if (!rc_connected_.load()) {
                RCLCPP_INFO(this->get_logger(), "RC connection established.");
            }
            rc_connected_.store(true);   // RC is connected

            uint16_t arm_channel_value = msg->channels[ARMING_CHANNEL_INDEX];
            uint16_t offboard_channel_value = msg->channels[OFFBOARD_CHANNEL_INDEX];
            
            // Check arming switch position
            if (arm_channel_value > ARM_CHANNEL_THRESHOLD) {
                // If rc was previously disarmed, log arming
                if (!rc_armed_.load()) {
                    RCLCPP_INFO(this->get_logger(), "RC arming switch moved to ARM position. Value: %d", arm_channel_value);
                }
                rc_armed_.store(true);
            } else {
                // If rc was previously armed, log disarming
                if (rc_armed_.load()) {
                    RCLCPP_INFO(this->get_logger(), "RC arming switch moved to DISARM position. Value: %d", arm_channel_value);
                }
                rc_armed_.store(false);
            }

            // Check offboard switch position
            if (offboard_channel_value > OFFBOARD_CHANNEL_THRESHOLD) {
                // If rc was previously in manual mode, log offboard mode
                if (!rc_offboard_.load()) {
                    RCLCPP_INFO(this->get_logger(), "RC offboard switch moved to OFFBOARD position. Value: %d", offboard_channel_value);
                }
                rc_offboard_.store(true);
            } else {
                // If rc was previously in offboard mode, log manual mode
                if (rc_offboard_.load()) {
                    RCLCPP_INFO(this->get_logger(), "RC offboard switch moved to MANUAL position. Value: %d", offboard_channel_value);
                }
                rc_offboard_.store(false);
            }
        } else {
            // If rc was previously connected, log disconnection
            if (rc_connected_.load()) {
                RCLCPP_WARN(this->get_logger(), "RC Channels message does not have expected number of channels. Expected: %d, Received: %zu", TOTAL_CHANNELS, msg->channels.size());
            }
            rc_connected_.store(false);
            // TODO: Decide logic if RC is disconnected mid-flight
            // rc_armed_ = false;
            // rc_offboard_ = false;
        }
    }

    void local_position_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg){
        {
            std::lock_guard<std::mutex> lock(current_pose_mutex_);
            current_pose_ = *msg;
        }
        if (!pose_received_.load()) {
            RCLCPP_INFO(this->get_logger(), "First local pose received.");
            pose_received_.store(true);
        }
    }

    void waypoint_path_callback(const nav_msgs::msg::Path::SharedPtr msg){
        if (!msg->poses.empty()) {
            std::lock_guard<std::mutex> lock(target_path_mutex_);
            target_path_ = *msg;
            RCLCPP_INFO(this->get_logger(), "Received waypoint path with %zu poses.", msg->poses.size());
        } else {
            RCLCPP_WARN(this->get_logger(), "Received empty waypoint path.");
        }
    }

    void setmode_response_callback(rclcpp::Client<mavros_msgs::srv::SetMode>::SharedFuture future){
        // Handle setmode response
        auto response = future.get();
        if (response->mode_sent) {
            offboard_mode_set_.store(true);
            RCLCPP_INFO(this->get_logger(), "Offboard request successful.");
        } else if (!response->mode_sent) {
            offboard_mode_set_.store(false);
            RCLCPP_ERROR(this->get_logger(), "Offboard request failed.");
        } else {
            offboard_mode_set_.store(false);
            RCLCPP_ERROR(this->get_logger(), "Offboard request returned unknown response.");
        }
    }

    void arming_response_callback(rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedFuture future){
        // Handle arming response
        auto response = future.get();
        if (response->success) {
            vehicle_armed_.store(true);
            RCLCPP_INFO(this->get_logger(), "Vehicle arm request successful. Result: %d", response->success);
        } else if (!response->success) {
            vehicle_armed_.store(false);
            RCLCPP_ERROR(this->get_logger(), "Vehicle arm request failed. Result: %d", response->success);
        } else {
            vehicle_armed_.store(false);
            RCLCPP_ERROR(this->get_logger(), "Vehicle arm request returned unknown response. Result: %d", response->success);
        }
    }

    void go_to_pose_goal_response_callback(
        const rclcpp_action::ClientGoalHandle<warehouse_drone_commander::action::GoToPose>::SharedPtr& goal_handle){
        // Handle goal response
        if (!goal_handle) {
            RCLCPP_ERROR(this->get_logger(), "GoToPose goal was rejected by server.");
            in_mission_.store(false);
            if (to_takeoff_.load()) {
                in_flight_.store(false);
            }
            return;
        }
        {
            // Set flags
            in_flight_.store(true);
            in_mission_.store(true);
        }
        RCLCPP_INFO(this->get_logger(), "GoToPose goal accepted by server, waiting for result...");
    }

    void go_to_pose_feedback_callback(
        const rclcpp_action::ClientGoalHandle<warehouse_drone_commander::action::GoToPose>::SharedPtr& goal_handle,
        const std::shared_ptr<const warehouse_drone_commander::action::GoToPose::Feedback>& feedback){
        // Handle feedback
        RCLCPP_INFO(this->get_logger(), "GoToPose Feedback: Distance remaining: %.2f m, Elapsed time: %.2f s",
                    feedback->distance_remaining,
                    feedback->elapsed_time);
        if (feedback->elapsed_time > go_to_pose_timeout_) {
            RCLCPP_WARN(this->get_logger(), "GoToPose action taking too long, cancelling action.");
            bool cancel_success = go_to_pose_cancel_goal(goal_handle);
            if (cancel_success) {
                RCLCPP_INFO(this->get_logger(), "GoToPose action cancellation requested successfully.");
            } else {
                RCLCPP_ERROR(this->get_logger(), "Failed to request GoToPose action cancellation.");
            }
        }
    }

    void go_to_pose_result_callback(
        const rclcpp_action::ClientGoalHandle<warehouse_drone_commander::action::GoToPose>::WrappedResult& result){
        // Handle result
        // TODO: Process result based on result.code
        switch (result.code) {
            case rclcpp_action::ResultCode::SUCCEEDED:
                // Further actions upon success
                {
                    if (to_takeoff_.load()) {
                        hold_position();    // Hold position after takeoff
                        to_takeoff_.store(false);
                        // TODO: Add logic
                    } else {
                        // TODO: Add logic
                    }
                }
                // Notify success
                RCLCPP_INFO(this->get_logger(), "GoToPose action succeeded.");
                break;

            case rclcpp_action::ResultCode::ABORTED:
                // Notify failure
                RCLCPP_ERROR(this->get_logger(), "GoToPose action was aborted.");
                return;

            case rclcpp_action::ResultCode::CANCELED:
                // Notify cancellation
                RCLCPP_WARN(this->get_logger(), "GoToPose action was canceled.");
                return;

            default:
                // Notify unknown result code
                RCLCPP_ERROR(this->get_logger(), "Unknown result code for GoToPose action.");
                return;

        }
        RCLCPP_INFO(this->get_logger(), "GoToPose Result: Success: %d", result.result->success);
    }

    void loop_timer_callback(){
        {
            if (!rc_connected_.load()) {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "RC not connected. Waiting for RC connection...");
                return;
            } else {
                // Update flags based on RC input
                if (!rc_armed_.load()) {
                    to_arm_vehicle_.store(false);
                } else {
                    to_arm_vehicle_.store(true);
                }
                if (!rc_offboard_.load()) {
                    to_set_offboard_.store(false);
                } else {
                    to_set_offboard_.store(true);
                }
                RCLCPP_DEBUG(this->get_logger(), "RC connected.");
            }
        }

        if (!startup_sequence_completed_.load()) {
            startup_sequence_init();
            return; // Exit the timer callback to allow startup sequence to complete first
        } else {
            // TODO: Main control logic after startup sequence is completed
        }
    }

    ////////////////////////////////////// Core Functions /////////////////////////////////////

    void startup_sequence_init(){
        // Implement startup sequence logic here
        RCLCPP_INFO(this->get_logger(), "Initiating startup sequence...");
        mavros_msgs::msg::State current_state_copy;
        {
            std::lock_guard<std::mutex> lock(current_state_mutex_);
            current_state_copy = current_state_;
        }
        if (state_received_.load() && current_state_copy.connected && rc_connected_.load() && pose_received_.load()) {

            // Publish setpoint - use current pose
            {
                std::lock_guard<std::mutex> lock(current_pose_mutex_);
                update_setpoint(current_pose_); // Update setpoint to current position (not taking off yet)
            }
            publish_setpoint();

            // Set offboard mode and arm the vehicle ONLY ONCE if flags are set
            // Set offboard mode if not already in offboard mode and user's RC requests it
            if (current_state_copy.mode != "OFFBOARD" && to_set_offboard_.load()) {
                auto offb_set_mode = std::make_shared<mavros_msgs::srv::SetMode::Request>();
                offb_set_mode->custom_mode = "OFFBOARD";
                if (set_mode_client_->service_is_ready()) {
                    auto set_mode_future = set_mode_client_->async_send_request(offb_set_mode, std::bind(&OffboardCommander::setmode_response_callback, this, _1));
                    RCLCPP_INFO(this->get_logger(), "Offboard request sent...");
                }
            } else if (current_state_copy.mode == "OFFBOARD" && offboard_mode_set_.load()) {
                // Arm the vehicle if not currently armed, and the flag to arm is true
                if (!current_state_copy.armed && to_arm_vehicle_.load()) {
                    auto arm_cmd = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
                    arm_cmd->value = true;
                    if (arming_client_->service_is_ready()) {
                        auto arming_future = arming_client_->async_send_request(arm_cmd, std::bind(&OffboardCommander::arming_response_callback, this, _1));
                        RCLCPP_INFO(this->get_logger(), "Vehicle arm request sent...");
                    }
                } else if (current_state_copy.armed && vehicle_armed_.load()) {
                    // Takeoff sequence here
                    if (to_takeoff_.load() && !in_flight_.load()) {
                        takeoff_hold_position();
                        startup_sequence_completed_.store(true); // Startup sequence completed after takeoff
                        RCLCPP_DEBUG(this->get_logger(), "Startup sequence completed.");
                    } else {
                        // Already armed, OR,
                        // to_arm_vehicle_ is false
                    }
                }
            } else {
                // Already in offboard mode, OR,
                // to_set_offboard_ is false
            }
        } else {
            // MAVROS state, RC connection, or local pose not received yet
            // Notify and wait
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "Waiting for MAVROS state or local pose...");
        }
    }

    // TODO: Use action server for setpoint following
    void takeoff_hold_position(){
        // Hold the current position by setting the setpoint to current position
        if (pose_received_.load() && to_takeoff_.load() && !in_flight_.load()) {
            geometry_msgs::msg::PoseStamped takeoff_pose;
            {
                std::lock_guard<std::mutex> lock(current_pose_mutex_);
                takeoff_pose = current_pose_;    // Create takeoff pose based on current pose
            }
            takeoff_pose.pose.position.z = takeoff_height_; // Modify only the z position for takeoff
            go_to_pose_send_goal(takeoff_pose); // Send takeoff goal to action server

            RCLCPP_INFO(this->get_logger(), "Takeoff committed to position: (%.2f, %.2f, %.2f), yaw: %.2f rad", 
                        takeoff_pose.pose.position.x,
                        takeoff_pose.pose.position.y,
                        takeoff_pose.pose.position.z,
                        quaternion_to_yaw(takeoff_pose.pose.orientation));
        } else {
            RCLCPP_WARN(this->get_logger(), "TAKEOFF COMMIT ABORTED... Cannot hold position: No pose received yet or already in flight.");
        }
        // Wait for MAVROS state
    }

    // TODO: Use action server for setpoint following
    void hold_position(){
        // Hold the current position by setting the setpoint to current position
        if (pose_received_.load()) {
            // If already holding position, just maintain the setpoint to avoid oscillations caused by rapidly updating setpoint
            if (!holding_position_.load() && !to_takeoff_.load()) {
                {
                    std::lock_guard<std::mutex> lock(current_pose_mutex_);
                    RCLCPP_INFO(this->get_logger(), "Now holding position: (%.2f, %.2f, %.2f), yaw: %.2f rad.", 
                                current_pose_.pose.position.x,
                                current_pose_.pose.position.y,
                                current_pose_.pose.position.z,
                                quaternion_to_yaw(current_pose_.pose.orientation));
                    update_setpoint(current_pose_); // Update setpoint to current position
                }
                publish_setpoint();
                holding_position_.store(true);
                in_mission_.store(false);
            } else if (holding_position_.load() && !to_takeoff_.load()) {
                std::lock_guard<std::mutex> lock(setpoint_to_send_mutex_);
                RCLCPP_INFO(this->get_logger(), "Already holding position: (%.2f, %.2f, %.2f), yaw: %.2f rad.", 
                            setpoint_to_send_.position.x,
                            setpoint_to_send_.position.y,
                            setpoint_to_send_.position.z,
                            setpoint_to_send_.yaw);    // Already holding position, just maintain the setpoint
            } else if (to_takeoff_.load()) {
                RCLCPP_INFO(this->get_logger(), "Now holding takeoff position at height: %.2f m.", takeoff_height_);
                holding_position_.store(true);
                in_mission_.store(false);
            } else {
                // Should not reach here
                RCLCPP_DEBUG(this->get_logger(), "Unexpected state in hold_position().");
                holding_position_.store(false);
                in_mission_.store(false);
            }
        } else {
            RCLCPP_WARN(this->get_logger(), "Cannot hold position: No pose received yet.");
            holding_position_.store(false);
            in_mission_.store(false);
        }
    }

    // TODO: Implement waypoint following logic
    void waypoint_handler(){
        // Handle waypoint following logic here
        // This function can be called within the loop_timer_callback or as needed
        // TODO: Implement waypoint following logic
        nav_msgs::msg::Path path_copy;
        {
            std::lock_guard<std::mutex> lock(target_path_mutex_);
            path_copy = target_path_;
        }
        if (path_copy.poses.empty()) {
            RCLCPP_WARN(this->get_logger(), "Empty path received. No waypoints to follow.");
            return;
        }
        for (const auto& pose_stamped : path_copy.poses) {
            RCLCPP_INFO(this->get_logger(), "Waypoint: (%.2f, %.2f, %.2f)", 
                        pose_stamped.pose.position.x, 
                        pose_stamped.pose.position.y, 
                        pose_stamped.pose.position.z);
            // Add logic to move to each waypoint
        }
    }

    ///////////////////////////////////// Utility Functions /////////////////////////////////////

    void update_setpoint(const geometry_msgs::msg::PoseStamped::SharedPtr& pose = nullptr){
        // Update the setpoint_to_send_ message with desired values

        // If a message is provided, use its values
        if (pose != nullptr) {
            std::lock_guard<std::mutex> lock(setpoint_to_send_mutex_);
            setpoint_to_send_ = populate_setpoint(pose);
        } else {
            RCLCPP_WARN(this->get_logger(), "update_setpoint() called without argument... No new pose provided, maintaining previous setpoint.");
        }
    }

    void publish_setpoint(){
        // Set appropriate header timestamp and publish. Lock to read setpoint safely.
        {
            std::lock_guard<std::mutex> lock(setpoint_to_send_mutex_);
            setpoint_to_send_.header.stamp = this->now();
            setpoint_pub_->publish(setpoint_to_send_);
        }
    }

    mavros_msgs::msg::PositionTarget populate_setpoint(const geometry_msgs::msg::PoseStamped::SharedPtr& pose){
        mavros_msgs::msg::PositionTarget setpoint_msg;

        // TODO: Check values
        setpoint_msg.header.frame_id = "base_link";    // TODO: parameterize this from yaml file
        setpoint_msg.coordinate_frame = mavros_msgs::msg::PositionTarget::FRAME_LOCAL_NED;
        setpoint_msg.type_mask =    mavros_msgs::msg::PositionTarget::IGNORE_AFX |
                                    mavros_msgs::msg::PositionTarget::IGNORE_AFY |
                                    mavros_msgs::msg::PositionTarget::IGNORE_AFZ;  // Use position, velocity, yaw, yaw_rate
        setpoint_msg.position.x = pose->pose.position.x;
        setpoint_msg.position.y = pose->pose.position.y;
        setpoint_msg.position.z = pose->pose.position.z;
        setpoint_msg.velocity.x = max_velocity_x;
        setpoint_msg.velocity.y = max_velocity_y;
        setpoint_msg.velocity.z = max_velocity_z;
        setpoint_msg.acceleration_or_force.x = 0;
        setpoint_msg.acceleration_or_force.y = 0;
        setpoint_msg.acceleration_or_force.z = 0;
        setpoint_msg.yaw = quaternion_to_yaw(pose->pose.orientation);
        setpoint_msg.yaw_rate = max_yaw_rate;
        
        return setpoint_msg;
    }

    void go_to_pose_send_goal(const geometry_msgs::msg::PoseStamped& target_pose){
        // Send a goal to the GoToPose action server
        RCLCPP_INFO(this->get_logger(), "Waiting for GoToPose server for 5s...");
        if (!go_to_pose_action_client_->wait_for_action_server(5s)) {
            RCLCPP_ERROR(this->get_logger(), "GoToPose action server not available after waiting");
            return;
        }

        auto goal_msg = warehouse_drone_commander::action::GoToPose::Goal();
        // No additional locking needed here as target_pose is passed by value
        goal_msg.goal_pose = target_pose;
        goal_msg.max_velocity_x = max_velocity_x;
        goal_msg.max_velocity_y = max_velocity_y;
        goal_msg.max_velocity_z = max_velocity_z;
        goal_msg.max_yaw_rate = max_yaw_rate;

        auto send_goal_options = rclcpp_action::Client<warehouse_drone_commander::action::GoToPose>::SendGoalOptions();
        send_goal_options.goal_response_callback = std::bind(&OffboardCommander::go_to_pose_goal_response_callback, this, _1);
        send_goal_options.feedback_callback = std::bind(&OffboardCommander::go_to_pose_feedback_callback, this, _1, _2);
        send_goal_options.result_callback = std::bind(&OffboardCommander::go_to_pose_result_callback, this, _1);

        RCLCPP_INFO(this->get_logger(), "Sending GoToPose goal to position: (%.2f, %.2f, %.2f), yaw: %.2f rad", 
                    target_pose.pose.position.x,
                    target_pose.pose.position.y,
                    target_pose.pose.position.z,
                    quaternion_to_yaw(target_pose.pose.orientation));
        (void)go_to_pose_action_client_->async_send_goal(goal_msg, send_goal_options);
    }

    bool go_to_pose_cancel_goal(rclcpp_action::ClientGoalHandle<warehouse_drone_commander::action::GoToPose>::SharedPtr& goal_handle){
        // Cancel the current goal
        RCLCPP_INFO(this->get_logger(), "Cancelling GoToPose goal...");
        if (goal_handle != nullptr && goal_handle->is_active()) {
            auto future_cancel = go_to_pose_action_client_->async_cancel_goal(goal_handle);
            if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future_cancel) != rclcpp::FutureReturnCode::SUCCESS)
            {
                RCLCPP_ERROR(get_logger(), "Failed to cancel goal");
                return false;
            }
            RCLCPP_INFO(this->get_logger(), "GoToPose goal cancel request sent.");
            return true;
        } else {
            RCLCPP_WARN(this->get_logger(), "No active GoToPose goal to cancel. Check goal handle.");
            return false;
        }
    }

    // Utility function to extract yaw from quaternion (output in radians)
    float quaternion_to_yaw(const geometry_msgs::msg::Quaternion &q) {
        // Yaw (z-axis rotation) from quaternion
        double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
        double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
        return static_cast<float>(std::atan2(siny_cosp, cosy_cosp));
    }

    ///////////////////////////////////// Parameters /////////////////////////////////////

    // Parameters with sensible defaults (brace initialization)
    float takeoff_height_{1.0f};
    float max_velocity_x{1.0f};
    float max_velocity_y{1.0f};
    float max_velocity_z{1.0f};
    float max_yaw_rate{0.5f};
    float go_to_pose_timeout_{60.0f};

    /////////////////////////////////////// ROS Interfaces /////////////////////////////////////

    rclcpp::Subscription<mavros_msgs::msg::RCIn>::SharedPtr rc_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr local_position_sub_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr waypoint_path_sub_;
    rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_sub_;
    rclcpp::Publisher<mavros_msgs::msg::PositionTarget>::SharedPtr setpoint_pub_;
    rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr arming_client_;
    rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr set_mode_client_;

    ///////////////////////////////// Action Clients /////////////////////////////////////

    rclcpp_action::Client<warehouse_drone_commander::action::GoToPose>::SharedPtr go_to_pose_action_client_;

    /////////////////////////////////////// Timers /////////////////////////////////////

    rclcpp::TimerBase::SharedPtr loop_timer_;

    ///////////////////////////////////// State Variables /////////////////////////////////////

    mavros_msgs::msg::State current_state_{};
    geometry_msgs::msg::PoseStamped current_pose_{};
    nav_msgs::msg::Path target_path_{};
    mavros_msgs::msg::PositionTarget setpoint_to_send_{};

    /////////////////////////////////////// Mutexes /////////////////////////////////////

    std::mutex current_state_mutex_;
    std::mutex current_pose_mutex_;
    std::mutex target_path_mutex_;
    std::mutex setpoint_to_send_mutex_;

    /////////////////////////////////////// Flags /////////////////////////////////////

    // TODO: Use brace initialization wherever possible throughout the codebase
    // Using std::atomic<bool> for flags accessed from multiple threads/callbacks

    std::atomic<bool> state_received_{false};
    std::atomic<bool> pose_received_{false};

    std::atomic<bool> rc_connected_{false};
    std::atomic<bool> rc_armed_{false};
    std::atomic<bool> rc_offboard_{false};

    std::atomic<bool> to_set_offboard_{false};
    std::atomic<bool> to_arm_vehicle_{false};
    std::atomic<bool> to_takeoff_{false};

    std::atomic<bool> startup_sequence_completed_{false};

    std::atomic<bool> offboard_mode_set_{false};
    std::atomic<bool> vehicle_armed_{false};
    std::atomic<bool> in_flight_{false};

    std::atomic<bool> holding_position_{false};
    std::atomic<bool> in_mission_{false};
};

int main(int argc, char *argv[])
{
	rclcpp::init(argc, argv);

    rclcpp::NodeOptions node_options;

    auto offboard_commander_node = std::make_shared<OffboardCommander>(node_options);
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(offboard_commander_node);

    RCLCPP_INFO(offboard_commander_node->get_logger(), "Starting Offboard Commander Node...");
	executor.spin();
    RCLCPP_INFO(offboard_commander_node->get_logger(), "Shutting down Offboard Commander Node...");

	rclcpp::shutdown();
	return 0;
}
