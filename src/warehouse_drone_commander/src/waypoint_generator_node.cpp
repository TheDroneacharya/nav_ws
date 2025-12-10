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

#include <warehouse_drone_interfaces/action/go_to_pose.hpp>

using namespace std::placeholders;
using namespace std::chrono_literals;

class WaypointGenerator : public rclcpp::Node
{
public:
    explicit WaypointGenerator(const rclcpp::NodeOptions& options)
    : Node("waypoint_generator", options)
    {
        // Initialization code here
        RCLCPP_INFO(this->get_logger(), "Starting waypoint gnenerator node...");

        // TODO: read horizontal and vertical distance arrays from yaml parameter file
        horizontal_distances_ = {1.0f, 2.0f, 3.0f}; // meters
        vertical_distances_ = {0.5f, 1.0f, 1.5f};   // meters


        // Callback groups
        auto commander_callback_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        auto action_callback_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

        rc_sub_ = this->create_subscription<mavros_msgs::msg::RCIn>("/mavros/rc/in", 10, std::bind(&WaypointGenerator::rc_in_callback, this, _1))
        local_position_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>("mavros/local_position/pose", 10, std::bind(&WaypointGenerator::local_position_callback, this, _1));
        endpoints_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>("offboard_commander/endpoint_path", 10, std::bind(&WaypointGenerator::endpoints_callback, this, _1));
        waypoint_path_pub_ = this->create_publisher<nav_msgs::msg::Path>("waypoint_generator/path", 10);

    }

private:

    ///////////////////////////////////// Callbacks /////////////////////////////////////

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

    ////////////////////////////////////// Core Functions /////////////////////////////////////

    

    ///////////////////////////////////// Utility Functions /////////////////////////////////////

    // Utility function to extract yaw from quaternion (output in radians)
    float quaternion_to_yaw(const geometry_msgs::msg::Quaternion &q) {
        // Yaw (z-axis rotation) from quaternion
        double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
        double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
        return static_cast<float>(std::atan2(siny_cosp, cosy_cosp));
    }

    ///////////////////////////////////// Parameters /////////////////////////////////////

    // Parameters with sensible defaults (brace initialization)
    std::vector<float> horizontal_distances_{1.0f, 2.0f, 3.0f}; // meters
    std::vector<float> vertical_distances_{0.5f, 1.0f, 1.5f};   // meters

    /////////////////////////////////////// ROS Interfaces /////////////////////////////////////

    rclcpp::Subscription<mavros_msgs::msg::RCIn>::SharedPtr rc_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr local_position_sub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr waypoint_path_pub_;

    ///////////////////////////////////// State Variables /////////////////////////////////////

    geometry_msgs::msg::PoseStamped current_pose_{};
    nav_msgs::msg::Path target_path_{};

    /////////////////////////////////////// Mutexes /////////////////////////////////////

    std::mutex current_pose_mutex_;
    std::mutex target_path_mutex_;

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
