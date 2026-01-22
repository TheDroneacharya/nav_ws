#include <functional>
#include <memory>
#include <thread>
#include <vector>
#include <string>
#include <cmath>
#include <algorithm>
#include <fstream>
#include <iostream>
#include <mutex>
#include <atomic>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"
#include "sensor_msgs/msg/battery_state.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "std_msgs/msg/bool.hpp"
#include "waypoint_interfaces/action/inspect_rack.hpp"
#include "waypoint_interfaces/action/return_to_launch.hpp"
#include "waypoint_interfaces/msg/mission_state.hpp"
#include "waypoint_interfaces/srv/resume_mission.hpp"
#include "yaml-cpp/yaml.h"
#include "ament_index_cpp/get_package_share_directory.hpp"

using namespace std::chrono_literals;

struct Point3D {
    double x, y, z;
};

class WaypointServer : public rclcpp::Node
{
public:
    using InspectRack = waypoint_interfaces::action::InspectRack;
    using ReturnToLaunch = waypoint_interfaces::action::ReturnToLaunch;
    using MissionState = waypoint_interfaces::msg::MissionState;
    using ResumeMission = waypoint_interfaces::srv::ResumeMission;
    using GoalHandleInspectRack = rclcpp_action::ServerGoalHandle<InspectRack>;
    using GoalHandleRTL = rclcpp_action::ServerGoalHandle<ReturnToLaunch>;

    explicit WaypointServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
    : Node("waypoint_server", options)
    , mission_state_(MissionState::STATE_IDLE)
    , current_waypoint_index_(0)
    , battery_percentage_(100.0)
    , system_healthy_(true)
    , rtl_triggered_(false)
    , wall_offset_y_(0.0)
    {
        // Declare parameters
        this->declare_parameter("config_file", "");
        this->declare_parameter("rtl.battery_rtl_threshold", 20.0);
        this->declare_parameter("rtl.battery_critical_threshold", 10.0);
        this->declare_parameter("rtl.rtl_altitude", 2.0);
        this->declare_parameter("rtl.enable_auto_rtl", true);
        this->declare_parameter("rtl.enable_auto_resume", true);
        this->declare_parameter("rtl.resume_battery_threshold", 50.0);
        this->declare_parameter("mission.state_publish_rate", 1.0);
        this->declare_parameter("mission.enable_wall_offset", true);
        this->declare_parameter("mission.max_wall_offset", 0.5);

        // Get parameters
        battery_rtl_threshold_ = this->get_parameter("rtl.battery_rtl_threshold").as_double();
        battery_critical_threshold_ = this->get_parameter("rtl.battery_critical_threshold").as_double();
        rtl_altitude_ = this->get_parameter("rtl.rtl_altitude").as_double();
        enable_auto_rtl_ = this->get_parameter("rtl.enable_auto_rtl").as_bool();
        enable_auto_resume_ = this->get_parameter("rtl.enable_auto_resume").as_bool();
        resume_battery_threshold_ = this->get_parameter("rtl.resume_battery_threshold").as_double();
        enable_wall_offset_ = this->get_parameter("mission.enable_wall_offset").as_bool();
        max_wall_offset_ = this->get_parameter("mission.max_wall_offset").as_double();

        // Initialize takeoff position
        takeoff_position_.x = 0.0;
        takeoff_position_.y = 0.0;
        takeoff_position_.z = 0.0;

        // Create InspectRack action server
        inspect_action_server_ = rclcpp_action::create_server<InspectRack>(
            this,
            "inspect_rack",
            std::bind(&WaypointServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&WaypointServer::handle_cancel, this, std::placeholders::_1),
            std::bind(&WaypointServer::handle_accepted, this, std::placeholders::_1)
        );

        // Create RTL action server
        rtl_action_server_ = rclcpp_action::create_server<ReturnToLaunch>(
            this,
            "return_to_launch",
            std::bind(&WaypointServer::handle_rtl_goal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&WaypointServer::handle_rtl_cancel, this, std::placeholders::_1),
            std::bind(&WaypointServer::handle_rtl_accepted, this, std::placeholders::_1)
        );

        // Create resume mission service
        resume_service_ = this->create_service<ResumeMission>(
            "resume_mission",
            std::bind(&WaypointServer::handle_resume_mission, this, 
                std::placeholders::_1, std::placeholders::_2)
        );

        // Create subscribers
        battery_sub_ = this->create_subscription<sensor_msgs::msg::BatteryState>(
            "/battery_state", 10,
            std::bind(&WaypointServer::battery_callback, this, std::placeholders::_1)
        );

        health_sub_ = this->create_subscription<diagnostic_msgs::msg::DiagnosticStatus>(
            "/system_health", 10,
            std::bind(&WaypointServer::health_callback, this, std::placeholders::_1)
        );

        wall_offset_sub_ = this->create_subscription<geometry_msgs::msg::Vector3Stamped>(
            "/wall_distance/offset", 10,
            std::bind(&WaypointServer::wall_offset_callback, this, std::placeholders::_1)
        );

        // Create publishers
        mission_state_pub_ = this->create_publisher<MissionState>("/mission_state", 10);

        // Create timer for publishing mission state
        double state_rate = this->get_parameter("mission.state_publish_rate").as_double();
        state_timer_ = this->create_wall_timer(
            std::chrono::duration<double>(1.0 / state_rate),
            std::bind(&WaypointServer::publish_mission_state, this)
        );

        RCLCPP_INFO(this->get_logger(), "Waypoint Server started with RTL & Wall Distance support");
        RCLCPP_INFO(this->get_logger(), "  RTL Battery Threshold: %.1f%%", battery_rtl_threshold_);
        RCLCPP_INFO(this->get_logger(), "  Wall Offset: %s", enable_wall_offset_ ? "Enabled" : "Disabled");
    }

private:
    // Action servers
    rclcpp_action::Server<InspectRack>::SharedPtr inspect_action_server_;
    rclcpp_action::Server<ReturnToLaunch>::SharedPtr rtl_action_server_;

    // Service
    rclcpp::Service<ResumeMission>::SharedPtr resume_service_;

    // Subscribers
    rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr battery_sub_;
    rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticStatus>::SharedPtr health_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr wall_offset_sub_;

    // Publishers
    rclcpp::Publisher<MissionState>::SharedPtr mission_state_pub_;

    // Timer
    rclcpp::TimerBase::SharedPtr state_timer_;

    // Mission state
    std::atomic<uint8_t> mission_state_;
    std::atomic<uint32_t> current_waypoint_index_;
    std::atomic<uint32_t> total_waypoints_;
    std::string current_rack_id_;
    std::string current_mission_id_;
    geometry_msgs::msg::Point takeoff_position_;
    geometry_msgs::msg::Point current_position_;
    geometry_msgs::msg::Point last_waypoint_;
    std::vector<Point3D> stored_waypoints_;
    rclcpp::Time mission_start_time_;
    std::mutex mission_mutex_;

    // Battery and health
    std::atomic<float> battery_percentage_;
    std::atomic<bool> system_healthy_;
    std::atomic<bool> rtl_triggered_;

    // Wall offset
    std::atomic<double> wall_offset_y_;

    // Parameters
    double battery_rtl_threshold_;
    double battery_critical_threshold_;
    double rtl_altitude_;
    bool enable_auto_rtl_;
    bool enable_auto_resume_;
    double resume_battery_threshold_;
    bool enable_wall_offset_;
    double max_wall_offset_;

    // === InspectRack Action Handlers ===
    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const InspectRack::Goal> goal)
    {
        RCLCPP_INFO(this->get_logger(), "Received goal request for rack_id: %s", goal->rack_id.c_str());
        (void)uuid;

        // Reject if RTL is in progress
        if (mission_state_ == MissionState::STATE_RTL) {
            RCLCPP_WARN(this->get_logger(), "Cannot accept goal - RTL in progress");
            return rclcpp_action::GoalResponse::REJECT;
        }

        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<GoalHandleInspectRack> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
        (void)goal_handle;
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted(const std::shared_ptr<GoalHandleInspectRack> goal_handle)
    {
        std::thread{std::bind(&WaypointServer::execute, this, std::placeholders::_1), goal_handle}.detach();
    }

    void execute(const std::shared_ptr<GoalHandleInspectRack> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Executing inspection goal");
        const auto goal = goal_handle->get_goal();
        auto feedback = std::make_shared<InspectRack::Feedback>();
        auto result = std::make_shared<InspectRack::Result>();

        // Store mission info
        {
            std::lock_guard<std::mutex> lock(mission_mutex_);
            current_rack_id_ = goal->rack_id;
            current_mission_id_ = std::to_string(this->now().nanoseconds());
            mission_start_time_ = this->now();
            
            // Store takeoff position as current (0,0,0) if not set
            takeoff_position_.x = 0.0;
            takeoff_position_.y = 0.0;
            takeoff_position_.z = 0.0;
        }

        mission_state_ = MissionState::STATE_RUNNING;
        rtl_triggered_ = false;

        // Load configuration and generate waypoints
        double wall_w = 5.0, wall_h = 5.0, focal_len = 50.0;
        double work_dist = 1.0, sensor_w = 36.0, sensor_h = 24.0, overlap = 0.1;

        if (!goal->rack_id.empty()) {
            try {
                std::string config_path = this->get_parameter("config_file").as_string();
                if (config_path.empty()) {
                    config_path = ament_index_cpp::get_package_share_directory("waypoint_generator") + "/config/rack_sample.yaml";
                }

                RCLCPP_INFO(this->get_logger(), "Loading config from: %s", config_path.c_str());
                
                YAML::Node config = YAML::LoadFile(config_path);
                if (config[goal->rack_id]) {
                    auto rack_node = config[goal->rack_id];
                    if (rack_node["wall_width"]) wall_w = rack_node["wall_width"].as<double>();
                    if (rack_node["wall_height"]) wall_h = rack_node["wall_height"].as<double>();
                    if (rack_node["camera"]) {
                        auto cam = rack_node["camera"];
                        if (cam["focal_length"]) focal_len = cam["focal_length"].as<double>();
                        if (cam["working_distance"]) work_dist = cam["working_distance"].as<double>();
                        if (cam["sensor_width"]) sensor_w = cam["sensor_width"].as<double>();
                        if (cam["sensor_height"]) sensor_h = cam["sensor_height"].as<double>();
                        if (cam["overlap"]) overlap = cam["overlap"].as<double>();
                    }
                }
            } catch (const std::exception &e) {
                RCLCPP_ERROR(this->get_logger(), "Failed to load config: %s", e.what());
            }
        }

        // Apply goal overrides
        if (goal->wall_width > 0) wall_w = goal->wall_width;
        if (goal->wall_height > 0) wall_h = goal->wall_height;
        if (goal->working_distance > 0) work_dist = goal->working_distance;
        
        // Generate waypoints
        double cov_w = (sensor_w * work_dist) / focal_len;
        double cov_h = (sensor_h * work_dist) / focal_len;
        double step_x = cov_w * (1.0 - overlap);
        double step_y = cov_h * (1.0 - overlap);

        int cols = (wall_w > cov_w) ? 1 + (int)std::ceil((wall_w - cov_w) / step_x) : 1;
        int rows = (wall_h > cov_h) ? 1 + (int)std::ceil((wall_h - cov_h) / step_y) : 1;

        double grid_span_x = cov_w + (cols - 1) * step_x;
        double grid_span_y = cov_h + (rows - 1) * step_y;
        double offset_x = (wall_w - grid_span_x) / 2.0;
        double offset_y = (wall_h - grid_span_y) / 2.0;
        double start_x = offset_x + (cov_w / 2.0);
        double start_y = offset_y + (cov_h / 2.0);

        std::vector<Point3D> centers;
        for (int r = 0; r < rows; ++r) {
            if (r % 2 == 0) {
                for (int c = 0; c < cols; ++c) {
                    centers.push_back({start_x + c * step_x, start_y + r * step_y, work_dist});
                }
            } else {
                for (int c = cols - 1; c >= 0; --c) {
                    centers.push_back({start_x + c * step_x, start_y + r * step_y, work_dist});
                }
            }
        }

        // Store waypoints for resume
        {
            std::lock_guard<std::mutex> lock(mission_mutex_);
            stored_waypoints_ = centers;
        }

        total_waypoints_ = centers.size();
        result->total_waypoints = centers.size();

        if (centers.empty()) {
            result->success = false;
            result->message = "No points generated";
            mission_state_ = MissionState::STATE_ERROR;
            goal_handle->abort(result);
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Generated %zu waypoints", centers.size());

        // Execute waypoints
        uint32_t start_idx = current_waypoint_index_.load();
        for (uint32_t i = start_idx; i < centers.size(); ++i) {
            // Check for cancellation
            if (goal_handle->is_canceling()) {
                result->success = false;
                result->message = "Goal canceled";
                mission_state_ = MissionState::STATE_PAUSED;
                goal_handle->canceled(result);
                return;
            }

            // Check battery and trigger RTL if needed
            if (enable_auto_rtl_ && should_trigger_rtl()) {
                RCLCPP_WARN(this->get_logger(), "RTL triggered! Battery: %.1f%%, Pausing at waypoint %d", 
                    battery_percentage_.load(), i);
                
                current_waypoint_index_ = i;
                mission_state_ = MissionState::STATE_RTL;
                rtl_triggered_ = true;
                
                result->success = false;
                result->message = "RTL triggered - mission paused";
                goal_handle->abort(result);
                return;
            }

            const auto& pt = centers[i];
            
            // Apply wall distance offset
            double adjusted_y = pt.y;
            if (enable_wall_offset_) {
                double offset = wall_offset_y_.load();
                offset = std::clamp(offset, -max_wall_offset_, max_wall_offset_);
                adjusted_y += offset;
            }

            geometry_msgs::msg::Point p;
            p.x = pt.x;
            p.y = adjusted_y;
            p.z = pt.z;

            // Update current position
            {
                std::lock_guard<std::mutex> lock(mission_mutex_);
                current_position_ = p;
                if (i > 0) {
                    last_waypoint_.x = centers[i-1].x;
                    last_waypoint_.y = centers[i-1].y;
                    last_waypoint_.z = centers[i-1].z;
                }
            }

            current_waypoint_index_ = i;
            feedback->current_waypoint = p;
            feedback->waypoint_index = i;
            feedback->percentage_complete = (float)(i + 1) / centers.size() * 100.0;
            
            goal_handle->publish_feedback(feedback);
            
            RCLCPP_INFO(this->get_logger(), "Waypoint %d/%zu: (%.2f, %.2f, %.2f) [offset: %.3f]", 
                        i + 1, centers.size(), p.x, p.y, p.z, wall_offset_y_.load());

            // 3.5 second delay per waypoint to simulate flight time
            std::this_thread::sleep_for(3500ms);
        }

        if (rclcpp::ok()) {
            result->success = true;
            result->message = "Mission completed successfully";
            mission_state_ = MissionState::STATE_COMPLETED;
            current_waypoint_index_ = 0;
            goal_handle->succeed(result);
            RCLCPP_INFO(this->get_logger(), "Mission completed!");
        }
    }

    // === RTL Action Handlers ===
    rclcpp_action::GoalResponse handle_rtl_goal(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const ReturnToLaunch::Goal> goal)
    {
        RCLCPP_INFO(this->get_logger(), "RTL goal received");
        (void)uuid;
        (void)goal;
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_rtl_cancel(
        const std::shared_ptr<GoalHandleRTL> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "RTL cancel request");
        (void)goal_handle;
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_rtl_accepted(const std::shared_ptr<GoalHandleRTL> goal_handle)
    {
        std::thread{std::bind(&WaypointServer::execute_rtl, this, std::placeholders::_1), goal_handle}.detach();
    }

    void execute_rtl(const std::shared_ptr<GoalHandleRTL> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Executing RTL");
        const auto goal = goal_handle->get_goal();
        auto feedback = std::make_shared<ReturnToLaunch::Feedback>();
        auto result = std::make_shared<ReturnToLaunch::Result>();

        mission_state_ = MissionState::STATE_RTL;

        geometry_msgs::msg::Point home = goal->takeoff_position;
        if (home.x == 0 && home.y == 0 && home.z == 0) {
            std::lock_guard<std::mutex> lock(mission_mutex_);
            home = takeoff_position_;
        }

        geometry_msgs::msg::Point current;
        {
            std::lock_guard<std::mutex> lock(mission_mutex_);
            current = current_position_;
        }

        // Simulate RTL sequence
        feedback->rtl_state = ReturnToLaunch::Feedback::RTL_STATE_ASCENDING;
        feedback->current_position = current;
        goal_handle->publish_feedback(feedback);
        std::this_thread::sleep_for(500ms);

        feedback->rtl_state = ReturnToLaunch::Feedback::RTL_STATE_FLYING_HOME;
        feedback->distance_to_home = std::sqrt(
            std::pow(home.x - current.x, 2) + 
            std::pow(home.y - current.y, 2) + 
            std::pow(home.z - current.z, 2)
        );
        goal_handle->publish_feedback(feedback);
        std::this_thread::sleep_for(500ms);

        feedback->rtl_state = ReturnToLaunch::Feedback::RTL_STATE_DESCENDING;
        feedback->current_position = home;
        feedback->distance_to_home = 0.0;
        feedback->percentage_complete = 100.0;
        goal_handle->publish_feedback(feedback);
        std::this_thread::sleep_for(500ms);

        feedback->rtl_state = ReturnToLaunch::Feedback::RTL_STATE_LANDED;
        goal_handle->publish_feedback(feedback);

        result->success = true;
        result->message = "RTL completed";
        result->final_position = home;
        
        mission_state_ = MissionState::STATE_PAUSED;
        goal_handle->succeed(result);

        RCLCPP_INFO(this->get_logger(), "RTL completed - mission can be resumed from waypoint %d", 
            current_waypoint_index_.load());
    }

    // === Resume Mission Service Handler ===
    void handle_resume_mission(
        const std::shared_ptr<ResumeMission::Request> request,
        std::shared_ptr<ResumeMission::Response> response)
    {
        RCLCPP_INFO(this->get_logger(), "Resume mission request received");

        if (mission_state_ != MissionState::STATE_PAUSED && 
            mission_state_ != MissionState::STATE_RTL) {
            response->success = false;
            response->message = "No paused mission to resume";
            return;
        }

        if (battery_percentage_ < resume_battery_threshold_) {
            response->success = false;
            response->message = "Battery too low to resume";
            return;
        }

        uint32_t resume_idx = request->start_from_waypoint;
        if (resume_idx == 0) {
            resume_idx = current_waypoint_index_.load();
        }

        current_waypoint_index_ = resume_idx;
        rtl_triggered_ = false;
        
        response->success = true;
        response->resumed_from_waypoint = resume_idx;
        response->remaining_waypoints = total_waypoints_ - resume_idx;
        response->message = "Mission ready to resume";

        RCLCPP_INFO(this->get_logger(), "Mission will resume from waypoint %d", resume_idx);
    }

    // === Subscription Callbacks ===
    void battery_callback(const sensor_msgs::msg::BatteryState::SharedPtr msg)
    {
        battery_percentage_ = msg->percentage * 100.0;
    }

    void health_callback(const diagnostic_msgs::msg::DiagnosticStatus::SharedPtr msg)
    {
        system_healthy_ = (msg->level == diagnostic_msgs::msg::DiagnosticStatus::OK);
    }

    void wall_offset_callback(const geometry_msgs::msg::Vector3Stamped::SharedPtr msg)
    {
        wall_offset_y_ = msg->vector.y;
    }

    // === Helper Functions ===
    bool should_trigger_rtl()
    {
        if (rtl_triggered_) return false;
        
        if (battery_percentage_ <= battery_rtl_threshold_) {
            return true;
        }
        if (!system_healthy_) {
            return true;
        }
        return false;
    }

    void publish_mission_state()
    {
        auto msg = MissionState();
        msg.header.stamp = this->now();
        
        {
            std::lock_guard<std::mutex> lock(mission_mutex_);
            msg.rack_id = current_rack_id_;
            msg.mission_id = current_mission_id_;
            msg.takeoff_position = takeoff_position_;
            msg.current_position = current_position_;
            msg.last_waypoint = last_waypoint_;
            msg.mission_start_time = mission_start_time_;
        }

        msg.total_waypoints = total_waypoints_;
        msg.current_waypoint_index = current_waypoint_index_;
        msg.percentage_complete = total_waypoints_ > 0 ? 
            (float)current_waypoint_index_ / total_waypoints_ * 100.0 : 0.0;
        msg.battery_percentage = battery_percentage_;
        msg.system_healthy = system_healthy_;
        msg.state = mission_state_;
        msg.last_update_time = this->now();
        msg.can_resume = (mission_state_ == MissionState::STATE_PAUSED);

        mission_state_pub_->publish(msg);
    }
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<WaypointServer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
