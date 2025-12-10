#include <chrono>
#include <functional>
#include <memory>
#include <cmath>
#include <atomic>
#include <mutex>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp/timer.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <mavros_msgs/msg/rc_in.hpp>
#include <mavros_msgs/msg/state.hpp>
#include <mavros_msgs/msg/position_target.hpp>
#include <mavros_msgs/srv/set_mode.hpp>
#include <mavros_msgs/srv/command_bool.hpp>

#include <warehouse_drone_interfaces/action/go_to_pose.hpp>

using namespace std::placeholders;
using namespace std::chrono_literals;

class OffboardCommander : public rclcpp::Node
{
public:
    explicit OffboardCommander(const rclcpp::NodeOptions& options) : Node("offboard_commander", options)
    {
        // Initialization code here
        RCLCPP_INFO(this->get_logger(), "Offboard commander node constructed...");

        // Parameter declarations and retrievals
        this->declare_parameter<std::string>("setpoint_frame_id", "base_link");
        this->declare_parameter<double>("takeoff_height", 1.0);
        this->declare_parameter<double>("max_velocity_x", 1.0);
        this->declare_parameter<double>("max_velocity_y", 1.0);
        this->declare_parameter<double>("max_velocity_z", 1.0);
        this->declare_parameter<double>("max_yaw_rate", 0.5);
        this->declare_parameter<double>("go_to_pose_timeout", 60.0);

        this->declare_parameter<int>("rc_arming_channel_index", 6); // Channel 6 is the arming switch (F)
        this->declare_parameter<int>("rc_offboard_channel_index", 11);  // Channel 11 is the offboard switch (A)
        this->declare_parameter<int>("rc_commandeered_channel_index", 12);  // Channel 12 is the commandeered switch (B)
        this->declare_parameter<int>("rc_total_channels", 18);  // Assuming 18 channels total
        this->declare_parameter<int>("rc_arm_channel_threshold", 1300); // Threshold value for arming switch
        this->declare_parameter<int>("rc_offboard_channel_threshold", 1500);    // Threshold value for offboard switch
        this->declare_parameter<int>("rc_commandeered_channel_threshold", 1500); // Threshold value for commandeered switch

        setpoint_frame_id_ = this->get_parameter("setpoint_frame_id").as_string();
        takeoff_height_ = this->get_parameter("takeoff_height").as_double();
        max_velocity_x = this->get_parameter("max_velocity_x").as_double();
        max_velocity_y = this->get_parameter("max_velocity_y").as_double();
        max_velocity_z = this->get_parameter("max_velocity_z").as_double();
        max_yaw_rate = this->get_parameter("max_yaw_rate").as_double();
        go_to_pose_timeout_ = this->get_parameter("go_to_pose_timeout").as_double();

        ARMING_CHANNEL_INDEX = this->get_parameter("rc_arming_channel_index").as_int() - 1; // Convert to 0-indexed
        OFFBOARD_CHANNEL_INDEX = this->get_parameter("rc_offboard_channel_index").as_int() - 1; // Convert to 0-indexed
        COMMANDEERED_CHANNEL_INDEX = this->get_parameter("rc_commandeered_channel_index").as_int() - 1; // Convert to 0-indexed
        TOTAL_CHANNELS = this->get_parameter("rc_total_channels").as_int();
        ARM_CHANNEL_THRESHOLD = this->get_parameter("rc_arm_channel_threshold").as_int();
        OFFBOARD_CHANNEL_THRESHOLD = this->get_parameter("rc_offboard_channel_threshold").as_int();
        COMMANDEERED_CHANNEL_THRESHOLD = this->get_parameter("rc_commandeered_channel_threshold").as_int();

        // Callback groups
        commander_callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        action_callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

        auto sensor_qos = rclcpp::SensorDataQoS();

        rc_sub_ = this->create_subscription<mavros_msgs::msg::RCIn>("/mavros/rc/in", sensor_qos, std::bind(&OffboardCommander::rc_in_callback, this, _1));
        state_sub_ = this->create_subscription<mavros_msgs::msg::State>("mavros/state", sensor_qos, std::bind(&OffboardCommander::state_callback, this, _1));
        local_position_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>("mavros/local_position/pose", sensor_qos, std::bind(&OffboardCommander::local_position_callback, this, _1));
        waypoint_target_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>("offboard_commander/input/waypoint_target", sensor_qos, std::bind(&OffboardCommander::waypoint_target_callback, this, _1));
        waypoint_path_sub_ = this->create_subscription<nav_msgs::msg::Path>("waypoint_generator/path", sensor_qos, std::bind(&OffboardCommander::waypoint_path_callback, this, _1));
        setpoint_pub_ = this->create_publisher<mavros_msgs::msg::PositionTarget>("offboard_commander/setpoint_raw/local", sensor_qos);
        arming_client_ = this->create_client<mavros_msgs::srv::CommandBool>("mavros/cmd/arming");
        set_mode_client_ = this->create_client<mavros_msgs::srv::SetMode>("mavros/set_mode");

        // Timers
        loop_timer_ = this->create_wall_timer(
            500ms,
            std::bind(&OffboardCommander::loop_timer_callback_v2, this),
            commander_callback_group_
        );  // 500ms = 2 Hz timer, callback group used to avoid service callbacks being in the same callback group as calling code

        // Action client
        go_to_pose_action_client_ = rclcpp_action::create_client<warehouse_drone_interfaces::action::GoToPose>(this, "go_to_pose", action_callback_group_);
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

        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "RC Channels received: No. of channels: %zu", msg->channels.size());
        if (msg->channels.size() == TOTAL_CHANNELS) {
            // If rc was previously disconnected, log connection established
            if (!rc_connected_.load()) {
                RCLCPP_INFO(this->get_logger(), "RC connection established.");
            }
            rc_connected_.store(true);   // RC is connected

            uint16_t arm_channel_value = msg->channels[ARMING_CHANNEL_INDEX];
            uint16_t offboard_channel_value = msg->channels[OFFBOARD_CHANNEL_INDEX];
            uint16_t commandeered_channel_value = msg->channels[COMMANDEERED_CHANNEL_INDEX];
            
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

            // Check commandeered switch position
            if (commandeered_channel_value > COMMANDEERED_CHANNEL_THRESHOLD) {
                // If rc was previously not_commandeered, log commandeered mode
                if (!rc_commandeered_.load()) {
                    RCLCPP_INFO(this->get_logger(), "RC commandeered switch moved to COMMANDEERED position. Value: %d", commandeered_channel_value);
                }
                rc_commandeered_.store(true);
            } else {
                // If rc was previously commandeered, log not_commandeered mode
                if (rc_commandeered_.load()) {
                    RCLCPP_INFO(this->get_logger(), "RC commandeered switch moved to NOT_COMMANDEERED position. Value: %d", commandeered_channel_value);
                }
                rc_commandeered_.store(false);
            }

        } else {            
            // If rc was previously connected, log disconnection
            if (rc_connected_.load()) {
                RCLCPP_ERROR(this->get_logger(), "RC ERROR: No. of channels changed after connection! Expected: %d, Received: %zu. This should not happen!", TOTAL_CHANNELS, msg->channels.size());
            }

            // If RC is being conncted for the first time, log error
            RCLCPP_ERROR(this->get_logger(), "RC ERROR: Unexpected no. of channels! Expected: %d, Received: %zu. Check param file!", TOTAL_CHANNELS, msg->channels.size());
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

    void waypoint_target_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg){
        // If no mission received, set mode to waypoint target mode.
        if (!mission_received_.load()) {
            mission_received_.store(true);

            // Check if first waypoint target
            if (!waypoint_target_received_.load()) {
                waypoint_target_received_.store(true);
                waypoint_updated_.store(true);

                RCLCPP_INFO(this->get_logger(), "First waypoint target received at position: (%.2f, %.2f, %.2f), yaw: %.2f rad...", 
                            msg->pose.position.x,
                            msg->pose.position.y,
                            msg->pose.position.z,
                            quaternion_to_yaw(msg->pose.orientation));
                {
                    std::lock_guard<std::mutex> lock(target_pose_mutex_);
                    target_pose_ = *msg;
                }

                geometry_msgs::msg::PoseStamped current_pose_copy;
                {
                    std::lock_guard<std::mutex> lock(current_pose_mutex_);
                    current_pose_copy = current_pose_;
                }

                target_pose_.pose.position.z = current_pose_copy.pose.position.z; // Maintain current altitude

            } else {
                RCLCPP_ERROR(this->get_logger(), "IMPOSSIBLE STATE!: waypoint_target_callback(): Not in mission, but subsequent waypoint target received. This should not happen. Check logic.");
            }
        } else {
            // Check if in waypoint target mode or waypoint path mode
            if (waypoint_target_received_.load()) {
                waypoint_updated_.store(true);
                RCLCPP_INFO(this->get_logger(), "Subsequent waypoint target received. Target updated to position: (%.2f, %.2f, %.2f), yaw: %.2f rad...", 
                            msg->pose.position.x,
                            msg->pose.position.y,
                            msg->pose.position.z,
                            quaternion_to_yaw(msg->pose.orientation));
                {
                    std::lock_guard<std::mutex> lock(target_pose_mutex_);
                    target_pose_ = *msg;
                }

                geometry_msgs::msg::PoseStamped current_pose_copy;
                {
                    std::lock_guard<std::mutex> lock(current_pose_mutex_);
                    current_pose_copy = current_pose_;
                }

                target_pose_.pose.position.z = current_pose_copy.pose.position.z; // Maintain current altitude

            } else if (waypoint_path_received_.load()) {
                RCLCPP_WARN(this->get_logger(), "INVALID USER INPUT!: Waypoint target received, but currently in waypoint path mission. Target will be ignored and deleted.");
            } else {
                RCLCPP_ERROR(this->get_logger(), "IMPOSSIBLE STATE!: In mission, but no waypoint target or path received. This should not happen. Check logic.");
            }
        }
    }

    // TODO: Implement waypoint mode select logic like above callback
    void waypoint_path_callback(const nav_msgs::msg::Path::SharedPtr msg){
        if (!msg->poses.empty()) {
            std::lock_guard<std::mutex> lock(target_path_mutex_);
            target_path_ = *msg;
            RCLCPP_INFO(this->get_logger(), "Received waypoint path with %zu poses.", msg->poses.size());
        } else {
            RCLCPP_ERROR(this->get_logger(), "Received empty waypoint path.");
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
        const rclcpp_action::ClientGoalHandle<warehouse_drone_interfaces::action::GoToPose>::SharedPtr& goal_handle){
        // Handle goal response
        if (!goal_handle) {
            RCLCPP_ERROR(this->get_logger(), "GoToPose goal was rejected by server.");
            in_mission_.store(false);
            if (to_takeoff_.load() && !in_flight_.load()) {
                in_flight_.store(false);
            } else if (to_takeoff_.load() && in_flight_.load()) {
                RCLCPP_ERROR(this->get_logger(), "ERROR CASE: go_to_pose_goal_response_callback(): to_takeoff is true, but, ALREADY IN FLIGHT! This should not happen. Check logic.");
            
            // // Needless to specify cases:
            
            // } else if (!to_takeoff_.load() && !in_flight_.load()) {
            //     in_flight_.store(false);
            // } else if (!to_takeoff_.load() && in_flight_.load()) {
            //     // in_flight_ remains true
            // } else {
            //     RCLCPP_ERROR(this->get_logger(), "IMPOSSIBLE CASE: This should not happen! Check logic.");

            }
            return;
        }
        {
            std::lock_guard<std::mutex> lock(current_goal_handle_mutex_);
            current_goal_handle_ = goal_handle;
        }
            // Set flags
            in_flight_.store(true);
            if (!to_takeoff_.load()) {
                in_mission_.store(true);
            }
        RCLCPP_INFO(this->get_logger(), "GoToPose goal accepted by server, waiting for result...");
    }

    void go_to_pose_feedback_callback(
        const rclcpp_action::ClientGoalHandle<warehouse_drone_interfaces::action::GoToPose>::SharedPtr& goal_handle,
        const std::shared_ptr<const warehouse_drone_interfaces::action::GoToPose::Feedback>& feedback){

        (void) goal_handle;

        geometry_msgs::msg::PoseStamped current_pose_copy;
        {
            std::lock_guard<std::mutex> lock(current_pose_mutex_);
            current_pose_copy = current_pose_;
        }
        RCLCPP_INFO(this->get_logger(), "GoToPose Feedback: Current Position: (%.2f, %.2f, %.2f), yaw: %.2f rad.", 
                    current_pose_copy.pose.position.x,
                    current_pose_copy.pose.position.y,
                    current_pose_copy.pose.position.z,
                    quaternion_to_yaw(current_pose_copy.pose.orientation));
        RCLCPP_INFO(this->get_logger(), "GoToPose Feedback: Goal Position: (%.2f, %.2f, %.2f), yaw: %.2f rad.", 
                    feedback->goal_pose.pose.position.x,
                    feedback->goal_pose.pose.position.y,
                    feedback->goal_pose.pose.position.z,
                    quaternion_to_yaw(feedback->goal_pose.pose.orientation));

        // Handle feedback
        RCLCPP_INFO(this->get_logger(), "GoToPose Feedback: Distance remaining: %.2f m, Elapsed time: %.2f s",
                    feedback->distance_remaining,
                    feedback->elapsed_time);

        // if (feedback->elapsed_time > go_to_pose_timeout_) {
        //     RCLCPP_WARN(this->get_logger(), "GoToPose action taking too long, cancelling action.");
        //     bool cancel_success = go_to_pose_cancel_goal(goal_handle);
        //     if (cancel_success) {
        //         RCLCPP_INFO(this->get_logger(), "GoToPose action cancellation requested successfully.");
        //         hold_position();
        //     } else {
        //         RCLCPP_ERROR(this->get_logger(), "Failed to request GoToPose action cancellation.");
        //     }
        // }
    }

    void go_to_pose_result_callback(
        const rclcpp_action::ClientGoalHandle<warehouse_drone_interfaces::action::GoToPose>::WrappedResult& result){
        // Handle result
        // TODO: Process result based on result.code
        switch (result.code) {
            case rclcpp_action::ResultCode::SUCCEEDED:
                // Further actions upon success
                if (to_takeoff_.load()) {
                    RCLCPP_INFO(this->get_logger(), "TAKEOFF SUCCESS: GoToPose takeoff action succeeded. Now committing to hold position...");

                    hold_position();    // Hold position after takeoff
                    to_takeoff_.store(false);

                    RCLCPP_INFO(this->get_logger(), "TAKEOFF HOLD POSITION COMPLETE: hold_position returned after successful takeoff.");
                    // TODO: Add logic
                } else {
                    // TODO: Add logic
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
        if (!check_rc_status()) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "RC not connected, skipping loop iteration...");
            return;
        }

        if (!startup_sequence_completed_.load()) {
            RCLCPP_INFO(this->get_logger(), "Startup sequence not completed, initiating...");

            startup_sequence_init();
            return; // Exit the timer callback to allow startup sequence to complete first

        } else if (to_takeoff_.load()) {
            if (!in_flight_.load()) {
                RCLCPP_ERROR(this->get_logger(), "ERROR CASE: Startup sequence completed, to_takeoff is true, but, NOT in flight! This should not happen. Check logic.");
            } else {
                RCLCPP_WARN(this->get_logger(), "Startup sequence completed, executing takeoff action...");
            }

        } else if (!in_flight_.load()) {
            RCLCPP_ERROR(this->get_logger(), "ERROR CASE: Startup sequence completed, to_takeoff is false, and, NOT in flight! This should not happen. Check logic.");

        } else {
            // TODO: Main control logic after startup sequence is completed
            RCLCPP_INFO(this->get_logger(), "Startup sequence completed, entering main control loop.");

            if (in_mission_.load()) {
                // If in mission, handle waypoint following
                RCLCPP_INFO(this->get_logger(), "In mission, handling waypoint following...");

                if (waypoint_target_received_.load()) {
                    // Handle waypoint target following
                    RCLCPP_INFO(this->get_logger(), "Waypoint target received is set, handling waypoint target following...");

                    if (waypoint_updated_.load()) {
                        RCLCPP_INFO(this->get_logger(), "Waypoint target updated, updating waypoint...");

                        // rclcpp_action::ClientGoalHandle<warehouse_drone_interfaces::action::GoToPose> current_goal_handle_copy;
                        // {
                        //     std::lock_guard<std::mutex> lock(current_goal_handle_mutex_);
                        //     current_goal_handle_copy = current_goal_handle_;
                        // }
                        // auto current_goal_handle_shared_ptr = std::make_shared<rclcpp_action::ClientGoalHandle<warehouse_drone_interfaces::action::GoToPose>>(current_goal_handle_copy);
                        // update_waypoint(current_goal_handle_shared_ptr);

                        rclcpp_action::ClientGoalHandle<warehouse_drone_interfaces::action::GoToPose>::SharedPtr current_goal_handle_copy;
                        {
                            std::lock_guard<std::mutex> lock(current_goal_handle_mutex_);
                            current_goal_handle_copy = current_goal_handle_;
                        }
                        update_waypoint(current_goal_handle_copy);

                    }
                } else if (waypoint_path_received_.load()) {
                    // TODO: Handle waypoint path following
                    RCLCPP_INFO(this->get_logger(), "Waypoint path received is set, handling waypoint path following...");

                    waypoint_handler();

                } else {
                    RCLCPP_ERROR(this->get_logger(), "In mission, but no waypoint target or path received. This should not happen. Check logic.");

                }
            } else {
                // If not in mission, hold position
                RCLCPP_INFO(this->get_logger(), "Not in mission, holding position...");

                hold_position();

            }
        }
    }

    void loop_timer_callback_v2(){
        RCLCPP_INFO(this->get_logger(), "Executing secondary loop timer callback.");

        // Check & interpret RC status
        if (!check_rc_status()) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "RC not connected, skipping loop iteration...");
            return;
        }

        // Check, run, & wait for startup
        if (!check_startup_status()) {
            RCLCPP_DEBUG(this->get_logger(), "loop_timer_callback_v2(): Startup sequence running...");
            return;
        }

        // TODO: Main control logic after startup sequence is completed
        RCLCPP_INFO(this->get_logger(), "Startup sequence completed, entering main control loop.");

        if (mission_received_.load()) {
            // If in mission, handle waypoint following
            RCLCPP_INFO(this->get_logger(), "In mission, handling waypoint following...");

            if (waypoint_target_received_.load()) {
                // Handle waypoint target following
                RCLCPP_INFO(this->get_logger(), "Waypoint target received is set, handling waypoint target following...");

                if (waypoint_updated_.load()) {
                    RCLCPP_INFO(this->get_logger(), "Waypoint target updated, updating waypoint...");

                    rclcpp_action::ClientGoalHandle<warehouse_drone_interfaces::action::GoToPose>::SharedPtr current_goal_handle_copy;
                    {
                        std::lock_guard<std::mutex> lock(current_goal_handle_mutex_);
                        current_goal_handle_copy = current_goal_handle_;
                    }
                    update_waypoint(current_goal_handle_copy);

                }
            } else if (waypoint_path_received_.load()) {
                // TODO: Handle waypoint path following
                RCLCPP_INFO(this->get_logger(), "Waypoint path received is set, handling waypoint path following...");

                waypoint_handler();

            } else {
                RCLCPP_ERROR(this->get_logger(), "In mission, but no waypoint target or path received. This should not happen. Check logic.");

            }
        } else {
            // If not in mission, hold position
            RCLCPP_INFO(this->get_logger(), "Not in mission, holding position...");

            hold_position();

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
        if (state_received_.load() && current_state_copy.connected && rc_connected_.load() && pose_received_.load() && !startup_sequence_completed_.load()) {
            RCLCPP_INFO(this->get_logger(), "MAVROS state received, RC connected, and local pose received. Proceeding with startup sequence...");

            // Publish setpoint - use current pose
            geometry_msgs::msg::PoseStamped current_pose_copy;
            {
                std::lock_guard<std::mutex> lock(current_pose_mutex_);
                current_pose_copy = current_pose_;
            }
            update_setpoint(std::make_shared<geometry_msgs::msg::PoseStamped>(current_pose_copy)); // Update setpoint to current position (not taking off yet)
            publish_setpoint();

            // Set offboard mode and arm the vehicle ONLY ONCE if flags are set
            // Set offboard mode if not already in offboard mode and user's RC requests it
            if (current_state_copy.mode != "OFFBOARD" && to_set_offboard_.load()) {
                RCLCPP_WARN(this->get_logger(), "Not in OFFBOARD mode & to_set_offboard is true. Setting OFFBOARD mode...");

                auto offb_set_mode = std::make_shared<mavros_msgs::srv::SetMode::Request>();
                offb_set_mode->custom_mode = "OFFBOARD";

                if (set_mode_client_->service_is_ready()) {
                    auto set_mode_future = set_mode_client_->async_send_request(offb_set_mode, std::bind(&OffboardCommander::setmode_response_callback, this, _1));
                    RCLCPP_WARN(this->get_logger(), "Offboard request sent...");
                }

            } else if (current_state_copy.mode == "OFFBOARD" && offboard_mode_set_.load()) {
                RCLCPP_INFO(this->get_logger(), "In OFFBOARD mode & offboard_mode_set is true. Proceeding to arm vehicle...");

                // Arm the vehicle if not currently armed, and the flag to arm is true
                if (!current_state_copy.armed && to_arm_vehicle_.load()) {
                    RCLCPP_WARN(this->get_logger(), "Vehicle not armed & to_arm_vehicle is true. Arming vehicle...");

                    auto arm_cmd = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
                    arm_cmd->value = true;

                    if (arming_client_->service_is_ready()) {
                        auto arming_future = arming_client_->async_send_request(arm_cmd, std::bind(&OffboardCommander::arming_response_callback, this, _1));
                        RCLCPP_WARN(this->get_logger(), "Vehicle arm request sent...");
                    }
                    
                } else if (current_state_copy.armed && vehicle_armed_.load()) {
                    RCLCPP_INFO(this->get_logger(), "Vehicle already armed & vehicle_armed is true. Proceeding to takeoff if requested...");

                    // Takeoff sequence here
                    if (to_takeoff_.load() && !in_flight_.load()) {
                        RCLCPP_WARN(this->get_logger(), "to_takeoff is true & not in flight. Initiating takeoff sequence...");

                        takeoff_hold_position();
                        startup_sequence_completed_.store(true); // Startup sequence completed after takeoff
                        RCLCPP_WARN(this->get_logger(), "Startup sequence completed.");

                    } else if (!to_takeoff_.load() && !in_flight_.load()) {
                        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 500, "to_takeoff is false & not in flight.");

                    } else if (to_takeoff_.load() && in_flight_.load()) {
                        RCLCPP_ERROR(this->get_logger(), "ERROR CASE: to_takeoff is true, but, ALREADY IN FLIGHT! This should not happen. Check logic.");

                    } else if (!to_takeoff_.load() && in_flight_.load()) {
                        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 500, "to_takeoff is false & already in flight.");

                    } else {
                        RCLCPP_ERROR(this->get_logger(), "IMPOSSIBLE CASE: This should not happen! Check logic.");
                    }

                } else if (current_state_copy.armed && !vehicle_armed_.load()) {
                    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 500, "VEHICLE MANUALLY ARMED: Vehicle is armed & vehicle_armed is false. NOT armed via offboard commander.");

                } else if (!current_state_copy.armed && !vehicle_armed_.load()) {
                    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 500, "VEHICLE NOT ARMED: Vehicle not armed & vehicle_armed is false.");

                } else if (!current_state_copy.armed && !to_arm_vehicle_.load()) {
                    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 500, "SWITCH DISARMED: Vehicle not armed & to_arm_vehicle is false. RC switch must be disarmed.");

                } else {
                    // Already armed, OR,
                    // to_arm_vehicle_ is false
                    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 500, "ARMING EDGE CASE: Unlikely case reached in arming logic. Check logic.");
                }

            } else if (current_state_copy.mode == "OFFBOARD" && !offboard_mode_set_.load()) {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 500, "OFFBOARD MODE MANUALLY SET: In OFFBOARD mode & offboard_mode_set is false. NOT set via offboard commander.");

            } else if (current_state_copy.mode != "OFFBOARD" && !offboard_mode_set_.load()) {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 500, "OFFBOARD MODE NOT SET: Not in OFFBOARD mode & offboard_mode_set is false.");

            } else if (current_state_copy.mode != "OFFBOARD" && !to_set_offboard_.load()) {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 500, "SWITCH MANUAL MODE: Not in OFFBOARD mode & to_set_offboard is false. RC switch must be in manual mode.");

            } else {
                // Already in offboard mode, OR,
                // to_set_offboard_ is false
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 500, "OFFBOARD EDGE CASE: Unlikely case reached in offboard mode logic. Check logic.");
            }

        } else {
            // MAVROS state, RC connection, or local pose not received yet
            // Notify and wait
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "Waiting for MAVROS state or local pose or RC connection... Specific cases ahead:");

            if (!state_received_.load()) {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "SPECIFIC CASE: MAVROS state not received yet");
            }
            if (!rc_connected_.load()) {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "SPECIFIC CASE: RC not connected yet");
            }
            if (!pose_received_.load()) {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "SPECIFIC CASE: Local pose not received yet");
            }
        }
    }

    void startup(){
        RCLCPP_INFO(this->get_logger(), "Initiating startup sequence...");

        if (check_drone_vitals_status()) {
            RCLCPP_DEBUG(this->get_logger(), "startup(): Drone vitals check passed.");

            if (set_drone_ready_to_arm()) {
                RCLCPP_DEBUG(this->get_logger(), "startup(): Drone ready to arm.");

                if (takeoff()) {
                    RCLCPP_DEBUG(this->get_logger(), "startup(): Drone takeoff successful.");

                } else {
                    RCLCPP_INFO(this->get_logger(), "Could not takeoff drone during this iteration! Retrying...");
                }
            } else {
                RCLCPP_INFO(this->get_logger(), "Could not set drone ready to arm in this iteration! Retrying...");
            }
        } else {
            RCLCPP_INFO(this->get_logger(), "Drone vitals check failed! Waiting for next iteration...");
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
            geometry_msgs::msg::PoseStamped current_pose_copy;
            {
                std::lock_guard<std::mutex> lock(current_pose_mutex_);
                current_pose_copy = current_pose_;
            }

            // If already holding position, just maintain the setpoint to avoid oscillations caused by rapidly updating setpoint
            if (!holding_position_.load() && !to_takeoff_.load()) {
                RCLCPP_INFO(this->get_logger(), "Now holding position: (%.2f, %.2f, %.2f), yaw: %.2f rad.", 
                            current_pose_copy.pose.position.x,
                            current_pose_copy.pose.position.y,
                            current_pose_copy.pose.position.z,
                            quaternion_to_yaw(current_pose_.pose.orientation));
                update_setpoint(std::make_shared<geometry_msgs::msg::PoseStamped>(current_pose_copy)); // Update setpoint to current position
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
                RCLCPP_INFO(this->get_logger(), "Now holding takeoff position at height: %.2f m. Actual height: %.2f m.",
                            takeoff_height_,
                            current_pose_copy.pose.position.z);
                holding_position_.store(true);
                in_mission_.store(false);
            } else {
                // Should not reach here
                RCLCPP_ERROR(this->get_logger(), "IMPOSSIBLE STATE!: Unexpected state in hold_position().");
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

    void update_waypoint(const rclcpp_action::ClientGoalHandle<warehouse_drone_interfaces::action::GoToPose>::SharedPtr& goal_handle){
        // Attempt to cancel previous goal
        RCLCPP_INFO(this->get_logger(), "Updating waypoint target, cancelling previous GoToPose goal...");
        bool cancel_success = go_to_pose_cancel_goal(goal_handle);
        if (cancel_success) {
            RCLCPP_INFO(this->get_logger(), "GoToPose action cancellation requested successfully.");
            hold_position();
        } else {
            RCLCPP_WARN(this->get_logger(), "update_waypoint(): Failed to request GoToPose action cancellation.");
        }
        // Send new goal
        geometry_msgs::msg::PoseStamped target_pose_copy;
        {
            std::lock_guard<std::mutex> lock(target_pose_mutex_);
            target_pose_copy = target_pose_;
        }
        go_to_pose_send_goal(target_pose_copy);

        waypoint_updated_.store(false);
    }
    
    ///////////////////////////////////// Utility Functions /////////////////////////////////////

    bool check_startup_status(){
        if (!startup_sequence_completed_.load()) {
            RCLCPP_INFO(this->get_logger(), "Startup sequence not completed, initiating...");

            startup();
            return false; // Exit to allow startup sequence to complete first

        } else if (to_takeoff_.load()) {
            if (!in_flight_.load()) {
                RCLCPP_ERROR(this->get_logger(), "ERROR CASE: Startup sequence completed, to_takeoff is true, but, NOT in flight! This should not happen. Check logic.");
            } else {
                RCLCPP_INFO(this->get_logger(), "Startup sequence completed, waiting for takeoff action to complete...");
            }
            return false;

        } else if (!in_flight_.load()) {
            RCLCPP_ERROR(this->get_logger(), "ERROR CASE: Startup sequence completed, to_takeoff is false, and, NOT in flight! This should not happen. Check logic.");
            return false;

        } else {
            RCLCPP_INFO(this->get_logger(), "Startup sequence completed, drone is in flight.");
            return true;
        }
    }

    bool check_rc_status(){
        if (!rc_connected_.load()) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "RC not connected. Waiting another iteration...");
            return false;
            
        } else {
            // Update flags based on RC input
            if (!startup_sequence_completed_.load()) {
                if (!rc_armed_.load()) {
                    to_arm_vehicle_.store(false);
                } else {
                    to_arm_vehicle_.store(true);
                }
                if (!rc_commandeered_.load()) {
                    to_set_offboard_.store(false);
                    to_takeoff_.store(false);   // TODO: Finalize logic for this flag
                } else {
                    to_set_offboard_.store(true);
                    to_takeoff_.store(true);    // TODO: Finalize logic for this flag
                }
            } else {
                // TODO: After startup sequence completed, RC input does different things (e.g., mission abort, return to home, etc.)
                RCLCPP_DEBUG(this->get_logger(), "check_rc_status(): Startup sequence completed, RC input not changing flags for now.");
            }
            RCLCPP_DEBUG(this->get_logger(), "RC connected.");

            return true;
        }
    }

    bool check_drone_vitals_status(){
        // Check various drone vitals and log warnings/errors as needed
        if (state_received_.load()) {
            mavros_msgs::msg::State current_state_copy;
            {
                std::lock_guard<std::mutex> lock(current_state_mutex_);
                current_state_copy = current_state_;
            }
            if (!current_state_copy.connected) {
                RCLCPP_ERROR(this->get_logger(), "DRONE VITALS ERROR: MAVROS not connected!");
                return false;
            }
            if (!rc_connected_.load()) {
                RCLCPP_ERROR(this->get_logger(), "DRONE VITALS ERROR: RC not connected!");
                return false;
            }
            if (!pose_received_.load()) {
                RCLCPP_ERROR(this->get_logger(), "DRONE VITALS ERROR: Local pose not received!");
                return false;
            }
            RCLCPP_INFO(this->get_logger(), "Drone vitals check: All vitals nominal.");
            return true;
        } else {
            RCLCPP_ERROR(this->get_logger(), "DRONE VITALS ERROR: MAVROS state NOT received!");
            return false;
        }
    }

    bool set_drone_ready_to_arm(){
        if (to_set_offboard_.load()) {
            RCLCPP_DEBUG(this->get_logger(), "set_drone_ready_to_arm(): to_set_offboard is true, proceeding to check vitals...");

            if (check_drone_vitals_status()) {
                RCLCPP_DEBUG(this->get_logger(), "set_drone_ready_to_arm(): Drone vitals check passed.");

                mavros_msgs::msg::State current_state_copy;
                {
                    std::lock_guard<std::mutex> lock(current_state_mutex_);
                    current_state_copy = current_state_;
                }

                // Set offboard mode if not already in offboard mode and user's RC requests it
                if (current_state_copy.mode != "OFFBOARD") {
                    RCLCPP_DEBUG(this->get_logger(), "set_drone_ready_to_arm(): Not in OFFBOARD mode, publishing current pose setpoint...");

                    geometry_msgs::msg::PoseStamped current_pose_copy;
                    {
                        std::lock_guard<std::mutex> lock(current_pose_mutex_);
                        current_pose_copy = current_pose_;
                    }
                    update_setpoint(std::make_shared<geometry_msgs::msg::PoseStamped>(current_pose_copy)); // Update setpoint to current position (not taking off yet)
                    publish_setpoint();

                    RCLCPP_DEBUG(this->get_logger(), "set_drone_ready_to_arm(): Current pose setpoint published. Sending OFFBOARD request...");

                    auto offb_set_mode = std::make_shared<mavros_msgs::srv::SetMode::Request>();
                    offb_set_mode->custom_mode = "OFFBOARD";

                    if (set_mode_client_->service_is_ready()) {
                        auto set_mode_future = set_mode_client_->async_send_request(offb_set_mode, std::bind(&OffboardCommander::setmode_response_callback, this, _1));
                        RCLCPP_INFO(this->get_logger(), "Offboard request sent...");

                        return true;

                    } else {
                        RCLCPP_WARN(this->get_logger(), "SetMode service not ready, cannot send OFFBOARD request.");
                        return false;
                    }
                } else {
                    RCLCPP_DEBUG(this->get_logger(), "set_drone_ready_to_arm(): Already in OFFBOARD mode, returning true.");
                    return true;
                }
            } else {
                RCLCPP_ERROR(this->get_logger(), "Drone not ready to be armed due to vitals check failure.");
                return false;
            }
        } else {
            RCLCPP_DEBUG(this->get_logger(), "set_drone_ready_to_arm(): to_set_offboard is false, returning false.");
            return false;
        }
    }

    bool arm(){
        if (to_arm_vehicle_.load()) {
            RCLCPP_DEBUG(this->get_logger(), "arm(): to_arm_vehicle is true, proceeding to check vitals...");

            if (check_drone_vitals_status()) {
                RCLCPP_DEBUG(this->get_logger(), "arm(): Drone vitals check passed.");

                mavros_msgs::msg::State current_state_copy;
                {
                    std::lock_guard<std::mutex> lock(current_state_mutex_);
                    current_state_copy = current_state_;
                }

                if (current_state_copy.mode == "OFFBOARD") {
                    RCLCPP_DEBUG(this->get_logger(), "arm(): In OFFBOARD mode, proceeding to arm vehicle...");

                    if (!current_state_copy.armed) {
                        RCLCPP_DEBUG(this->get_logger(), "arm(): Vehicle not armed, sending arm request...");

                        auto arm_cmd = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
                        arm_cmd->value = true;

                        if (arming_client_->service_is_ready()) {
                            auto arming_future = arming_client_->async_send_request(arm_cmd, std::bind(&OffboardCommander::arming_response_callback, this, _1));
                            RCLCPP_INFO(this->get_logger(), "Vehicle arm request sent...");

                            return true;

                        } else {
                            RCLCPP_WARN(this->get_logger(), "Arming service not ready, cannot send arm request.");
                            return false;
                        }
                    } else {
                        RCLCPP_DEBUG(this->get_logger(), "arm(): Vehicle already armed, returning true.");
                        return true;
                    }
                } else {
                    RCLCPP_INFO(this->get_logger(), "Not in OFFBOARD mode yet, cannot takeoff. Waiting another iteration...");
                    return false;
                }
            } else {
                RCLCPP_ERROR(this->get_logger(), "Drone not ready to takeoff due to vitals check failure.");
                return false;
            }
        } else {
            RCLCPP_DEBUG(this->get_logger(), "arm(): to_arm_vehicle is false, returning false.");
            return false;
        }
    }

    bool takeoff(){
        // Takeoff procedure
        if (to_takeoff_.load()) {
            RCLCPP_DEBUG(this->get_logger(), "takeoff(): to_takeoff is true, proceeding to check vitals...");

            if (check_drone_vitals_status()) {
                RCLCPP_DEBUG(this->get_logger(), "takeoff(): Drone vitals check passed.");

                mavros_msgs::msg::State current_state_copy;
                {
                    std::lock_guard<std::mutex> lock(current_state_mutex_);
                    current_state_copy = current_state_;
                }

                if (arm()) {
                    RCLCPP_DEBUG(this->get_logger(), "takeoff(): Drone armed successfully, proceeding to takeoff...");

                    if (current_state_copy.armed) {
                        RCLCPP_DEBUG(this->get_logger(), "takeoff(): Vehicle is armed, executing takeoff sequence...");

                        // Takeoff sequence here
                        if (!in_flight_.load()) {
                            RCLCPP_DEBUG(this->get_logger(), "takeoff(): Not in flight, committing to takeoff...");

                            takeoff_hold_position();

                            startup_sequence_completed_.store(true);    // Startup sequence completed after takeoff committed
                            RCLCPP_INFO(this->get_logger(), "Startup sequence completed.");

                            return true;

                        } else {
                            RCLCPP_WARN(this->get_logger(), "Already in flight, cannot takeoff again.");
                            return false;
                        }
                    } else {
                        RCLCPP_INFO(this->get_logger(), "Vehicle not yet armed, cannot takeoff. Waiting another iteration...");
                        return false;
                    }
                } else {
                    RCLCPP_INFO(this->get_logger(), "Drone arming failed, cannot takeoff. Waiting another iteration...");
                    return false;
                }
            } else {
                RCLCPP_ERROR(this->get_logger(), "Drone not ready to takeoff due to vitals check failure.");
                return false;
            }
        } else {
            RCLCPP_DEBUG(this->get_logger(), "takeoff(): to_takeoff is false, returning false.");
            return false;
        }
    }

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
        setpoint_msg.header.frame_id = setpoint_frame_id_;
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
        RCLCPP_DEBUG(this->get_logger(), "go_to_pose_send_goal(): Received request to send GoToPose goal...");
        // Send a goal to the GoToPose action server
        RCLCPP_DEBUG(this->get_logger(), "go_to_pose_send_goal(): Waiting for GoToPose server for upto 5s...");
        if (!go_to_pose_action_client_->wait_for_action_server(5s)) {
            RCLCPP_ERROR(this->get_logger(), "GoToPose action server not available after waiting");
            return;
        }

        auto goal_msg = warehouse_drone_interfaces::action::GoToPose::Goal();
        // No additional locking needed here as target_pose is passed by value to this function
        goal_msg.goal_pose = target_pose;
        goal_msg.max_velocity_x = max_velocity_x;
        goal_msg.max_velocity_y = max_velocity_y;
        goal_msg.max_velocity_z = max_velocity_z;
        goal_msg.max_yaw_rate = max_yaw_rate;

        auto send_goal_options = rclcpp_action::Client<warehouse_drone_interfaces::action::GoToPose>::SendGoalOptions();
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

    bool go_to_pose_cancel_goal(const rclcpp_action::ClientGoalHandle<warehouse_drone_interfaces::action::GoToPose>::SharedPtr& goal_handle){
        RCLCPP_DEBUG(this->get_logger(), "go_to_pose_cancel_goal(): received request to cancel GoToPose goal...");
        if (!goal_handle) {
            RCLCPP_ERROR(this->get_logger(), "go_to_pose_cancel_goal(): Called with null goal_handle");
            return false;
        }
        // Cancel the current goal
        RCLCPP_DEBUG(this->get_logger(), "go_to_pose_cancel_goal(): attempting to cancel GoToPose goal...");
        int status = goal_handle->get_status();
        RCLCPP_INFO(this->get_logger(), "Current GoToPose goal status: %d", status);
        if (goal_handle != nullptr && (status == rclcpp_action::GoalStatus::STATUS_UNKNOWN ||
                                        status == rclcpp_action::GoalStatus::STATUS_ACCEPTED ||
                                        status == rclcpp_action::GoalStatus::STATUS_EXECUTING)) {
            auto future_cancel = go_to_pose_action_client_->async_cancel_goal(goal_handle);
            // TODO: Causes problems when spinning, look for alternatives
            // if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future_cancel) != rclcpp::FutureReturnCode::SUCCESS) {
            //     RCLCPP_ERROR(get_logger(), "Failed to cancel goal");
            //     return false;
            // }
            RCLCPP_INFO(this->get_logger(), "GoToPose goal cancel request sent.");
            return true;
        } else {
            RCLCPP_WARN(this->get_logger(), "No active GoToPose goal to cancel. Check goal handle.");
            return false;
        }
    }

    // Utility function to extract yaw from quaternion (output in radians)
    double quaternion_to_yaw(const geometry_msgs::msg::Quaternion &q) {
        // Yaw (z-axis rotation) from quaternion
        double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
        double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
        return std::atan2(siny_cosp, cosy_cosp);
    }

    ///////////////////////////////////// Parameters /////////////////////////////////////

    // Parameters with sensible defaults (brace initialization)
    std::string setpoint_frame_id_{};
    double takeoff_height_{};
    double max_velocity_x{};
    double max_velocity_y{};
    double max_velocity_z{};
    double max_yaw_rate{};
    double go_to_pose_timeout_{};

    unsigned int ARMING_CHANNEL_INDEX{};
    unsigned int OFFBOARD_CHANNEL_INDEX{};
    unsigned int COMMANDEERED_CHANNEL_INDEX{};
    unsigned int TOTAL_CHANNELS{};
    unsigned int ARM_CHANNEL_THRESHOLD{};
    unsigned int OFFBOARD_CHANNEL_THRESHOLD{};
    unsigned int COMMANDEERED_CHANNEL_THRESHOLD{};

    /////////////////////////////////////// Callback Groups /////////////////////////////////////

    rclcpp::CallbackGroup::SharedPtr commander_callback_group_{};
    rclcpp::CallbackGroup::SharedPtr action_callback_group_{};

    /////////////////////////////////////// ROS Interfaces /////////////////////////////////////

    rclcpp::Subscription<mavros_msgs::msg::RCIn>::SharedPtr rc_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr local_position_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr waypoint_target_sub_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr waypoint_path_sub_;
    rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_sub_;
    rclcpp::Publisher<mavros_msgs::msg::PositionTarget>::SharedPtr setpoint_pub_;
    rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr arming_client_;
    rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr set_mode_client_;

    ///////////////////////////////// Action Clients /////////////////////////////////////

    rclcpp_action::Client<warehouse_drone_interfaces::action::GoToPose>::SharedPtr go_to_pose_action_client_;

    /////////////////////////////////////// Timers /////////////////////////////////////

    rclcpp::TimerBase::SharedPtr loop_timer_;

    ///////////////////////////////////// State Variables /////////////////////////////////////

    mavros_msgs::msg::State current_state_{};
    geometry_msgs::msg::PoseStamped current_pose_{};
    geometry_msgs::msg::PoseStamped target_pose_{};
    nav_msgs::msg::Path target_path_{};
    mavros_msgs::msg::PositionTarget setpoint_to_send_{};
    rclcpp_action::ClientGoalHandle<warehouse_drone_interfaces::action::GoToPose>::SharedPtr current_goal_handle_;

    /////////////////////////////////////// Mutexes /////////////////////////////////////

    std::mutex current_state_mutex_;
    std::mutex current_pose_mutex_;
    std::mutex target_pose_mutex_;
    std::mutex target_path_mutex_;
    std::mutex setpoint_to_send_mutex_;
    std::mutex current_goal_handle_mutex_;

    /////////////////////////////////////// Flags /////////////////////////////////////

    // TODO: Use brace initialization wherever possible throughout the codebase
    // Using std::atomic<bool> for flags accessed from multiple threads/callbacks

    std::atomic<bool> state_received_{false};
    std::atomic<bool> pose_received_{false};

    std::atomic<bool> waypoint_target_received_{false};     // set to true when a new waypoint target is received, never unset
    std::atomic<bool> waypoint_updated_{false};             // set to true when a new waypoint target is received, unset after updating waypoint
    std::atomic<bool> waypoint_path_received_{false};       // set to true when a new waypoint path is received, never unset
    std::atomic<bool> mission_received_{false};             // set to true when a new mission is received, never unset

    std::atomic<bool> rc_connected_{false};
    std::atomic<bool> rc_armed_{false};
    std::atomic<bool> rc_offboard_{false};
    std::atomic<bool> rc_commandeered_{false};

    std::atomic<bool> to_set_offboard_{false};
    std::atomic<bool> to_arm_vehicle_{false};
    std::atomic<bool> to_takeoff_{false};                   // set via RC input, unset only after takeoff action positive result

    std::atomic<bool> startup_sequence_completed_{false};

    std::atomic<bool> offboard_mode_set_{false};
    std::atomic<bool> vehicle_armed_{false};
    std::atomic<bool> in_flight_{false};                    // set on positive action response, unset on negative action response ,TODO: unset on land

    std::atomic<bool> holding_position_{false};
    std::atomic<bool> in_mission_{false};                   // set on positive action response & to_takeoff_ is unset, unset on negative action response, TODO: unset on mission completion
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
