#include <memory>
#include <chrono>
#include <functional>
#include <string>
#include <mutex>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/range.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/bool.hpp"
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>

#include "wall_distance_controller/moving_average_filter.hpp"

using namespace std::chrono_literals;

namespace wall_distance_controller
{

/**
 * @brief Wall Distance Controller Node
 * 
 * Subscribes to depth camera data (Image or Range messages),
 * processes it with a moving average filter, and publishes
 * lateral offset corrections to maintain constant wall distance.
 */
class WallDistanceControllerNode : public rclcpp::Node
{
public:
    WallDistanceControllerNode()
        : Node("wall_distance_controller")
        , depth_filter_(10, true)  // 10-sample window with outlier rejection
        , controller_enabled_(true)
    {
        // Declare parameters
        this->declare_parameter("target_distance", 1.0);      // Target wall distance (m)
        this->declare_parameter("distance_tolerance", 0.1);    // Acceptable deviation (m)
        this->declare_parameter("max_correction", 0.3);        // Max lateral correction per update (m)
        this->declare_parameter("filter_window_size", 10);     // Moving average window
        this->declare_parameter("update_rate", 10.0);          // Control loop frequency (Hz)
        this->declare_parameter("min_valid_distance", 0.1);    // Minimum valid depth reading (m)
        this->declare_parameter("max_valid_distance", 10.0);   // Maximum valid depth reading (m)
        this->declare_parameter("depth_image_topic", "/depth_camera/depth");
        this->declare_parameter("range_topic", "/depth_camera/range");
        this->declare_parameter("use_depth_image", true);      // If false, use Range message instead
        this->declare_parameter("roi_center_x", 0.5);          // ROI center as fraction of image width
        this->declare_parameter("roi_center_y", 0.5);          // ROI center as fraction of image height
        this->declare_parameter("roi_width", 0.3);             // ROI width as fraction of image
        this->declare_parameter("roi_height", 0.3);            // ROI height as fraction of image
        this->declare_parameter("depth_scale", 0.001);         // Scale factor for depth values (mm to m)
        this->declare_parameter("kp", 0.5);                    // Proportional gain for control

        // Get parameters
        target_distance_ = this->get_parameter("target_distance").as_double();
        distance_tolerance_ = this->get_parameter("distance_tolerance").as_double();
        max_correction_ = this->get_parameter("max_correction").as_double();
        min_valid_distance_ = this->get_parameter("min_valid_distance").as_double();
        max_valid_distance_ = this->get_parameter("max_valid_distance").as_double();
        use_depth_image_ = this->get_parameter("use_depth_image").as_bool();
        roi_center_x_ = this->get_parameter("roi_center_x").as_double();
        roi_center_y_ = this->get_parameter("roi_center_y").as_double();
        roi_width_ = this->get_parameter("roi_width").as_double();
        roi_height_ = this->get_parameter("roi_height").as_double();
        depth_scale_ = this->get_parameter("depth_scale").as_double();
        kp_ = this->get_parameter("kp").as_double();

        int window_size = this->get_parameter("filter_window_size").as_int();
        depth_filter_ = MovingAverageFilter(window_size, true);

        double update_rate = this->get_parameter("update_rate").as_double();
        
        // Create subscriptions
        if (use_depth_image_) {
            std::string depth_topic = this->get_parameter("depth_image_topic").as_string();
            depth_image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
                depth_topic, 10,
                std::bind(&WallDistanceControllerNode::depthImageCallback, this, std::placeholders::_1)
            );
            RCLCPP_INFO(this->get_logger(), "Subscribing to depth image: %s", depth_topic.c_str());
        } else {
            std::string range_topic = this->get_parameter("range_topic").as_string();
            range_sub_ = this->create_subscription<sensor_msgs::msg::Range>(
                range_topic, 10,
                std::bind(&WallDistanceControllerNode::rangeCallback, this, std::placeholders::_1)
            );
            RCLCPP_INFO(this->get_logger(), "Subscribing to range: %s", range_topic.c_str());
        }

        // Enable/disable subscription
        enable_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "wall_distance/enable", 10,
            std::bind(&WallDistanceControllerNode::enableCallback, this, std::placeholders::_1)
        );

        // Create publishers
        offset_pub_ = this->create_publisher<geometry_msgs::msg::Vector3Stamped>(
            "wall_distance/offset", 10
        );
        
        current_distance_pub_ = this->create_publisher<std_msgs::msg::Float32>(
            "wall_distance/current_distance", 10
        );
        
        filtered_distance_pub_ = this->create_publisher<std_msgs::msg::Float32>(
            "wall_distance/filtered_distance", 10
        );

        status_pub_ = this->create_publisher<std_msgs::msg::Bool>(
            "wall_distance/filter_ready", 10
        );

        // Control loop timer
        auto period = std::chrono::duration<double>(1.0 / update_rate);
        control_timer_ = this->create_wall_timer(
            std::chrono::duration_cast<std::chrono::milliseconds>(period),
            std::bind(&WallDistanceControllerNode::controlLoop, this)
        );

        RCLCPP_INFO(this->get_logger(), 
            "Wall Distance Controller initialized. Target: %.2fm, Tolerance: %.2fm", 
            target_distance_, distance_tolerance_);
    }

private:
    /**
     * @brief Process depth image to extract wall distance
     */
    void depthImageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        if (!controller_enabled_) return;

        try {
            cv_bridge::CvImagePtr cv_ptr;
            
            // Handle different depth image encodings
            if (msg->encoding == "32FC1") {
                cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1);
            } else if (msg->encoding == "16UC1") {
                cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1);
            } else {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                    "Unsupported depth encoding: %s", msg->encoding.c_str());
                return;
            }

            cv::Mat depth_image = cv_ptr->image;
            
            // Calculate ROI bounds
            int img_width = depth_image.cols;
            int img_height = depth_image.rows;
            
            int roi_x = static_cast<int>((roi_center_x_ - roi_width_ / 2.0) * img_width);
            int roi_y = static_cast<int>((roi_center_y_ - roi_height_ / 2.0) * img_height);
            int roi_w = static_cast<int>(roi_width_ * img_width);
            int roi_h = static_cast<int>(roi_height_ * img_height);
            
            // Clamp ROI to image bounds
            roi_x = std::max(0, std::min(roi_x, img_width - 1));
            roi_y = std::max(0, std::min(roi_y, img_height - 1));
            roi_w = std::min(roi_w, img_width - roi_x);
            roi_h = std::min(roi_h, img_height - roi_y);

            cv::Rect roi(roi_x, roi_y, roi_w, roi_h);
            cv::Mat roi_depth = depth_image(roi);

            // Find minimum valid distance in ROI (closest obstacle)
            double min_distance = std::numeric_limits<double>::max();
            int valid_count = 0;
            double sum = 0.0;

            for (int y = 0; y < roi_depth.rows; ++y) {
                for (int x = 0; x < roi_depth.cols; ++x) {
                    double depth_val;
                    
                    if (msg->encoding == "32FC1") {
                        depth_val = roi_depth.at<float>(y, x);
                    } else {  // 16UC1
                        depth_val = roi_depth.at<uint16_t>(y, x) * depth_scale_;
                    }

                    // Check if value is valid
                    if (!std::isnan(depth_val) && !std::isinf(depth_val) &&
                        depth_val >= min_valid_distance_ && depth_val <= max_valid_distance_) {
                        if (depth_val < min_distance) {
                            min_distance = depth_val;
                        }
                        sum += depth_val;
                        valid_count++;
                    }
                }
            }

            if (valid_count > 0) {
                // Use minimum distance as the wall distance (closest point approach)
                std::lock_guard<std::mutex> lock(filter_mutex_);
                last_raw_distance_ = min_distance;
                depth_filter_.addSample(min_distance);
                
                // Publish raw distance
                auto raw_msg = std_msgs::msg::Float32();
                raw_msg.data = min_distance;
                current_distance_pub_->publish(raw_msg);
            }
            
        } catch (const cv_bridge::Exception& e) {
            RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                "cv_bridge exception: %s", e.what());
        }
    }

    /**
     * @brief Process range message for wall distance
     */
    void rangeCallback(const sensor_msgs::msg::Range::SharedPtr msg)
    {
        if (!controller_enabled_) return;

        double range = msg->range;
        
        // Validate range
        if (range >= msg->min_range && range <= msg->max_range &&
            range >= min_valid_distance_ && range <= max_valid_distance_) {
            
            std::lock_guard<std::mutex> lock(filter_mutex_);
            last_raw_distance_ = range;
            depth_filter_.addSample(range);
            
            // Publish raw distance
            auto raw_msg = std_msgs::msg::Float32();
            raw_msg.data = range;
            current_distance_pub_->publish(raw_msg);
        }
    }

    /**
     * @brief Enable/disable controller callback
     */
    void enableCallback(const std_msgs::msg::Bool::SharedPtr msg)
    {
        controller_enabled_ = msg->data;
        if (!controller_enabled_) {
            // Reset filter when disabled
            std::lock_guard<std::mutex> lock(filter_mutex_);
            depth_filter_.reset();
        }
        RCLCPP_INFO(this->get_logger(), "Controller %s", 
            controller_enabled_ ? "enabled" : "disabled");
    }

    /**
     * @brief Main control loop - computes and publishes offset corrections
     */
    void controlLoop()
    {
        if (!controller_enabled_) return;

        std::lock_guard<std::mutex> lock(filter_mutex_);

        // Publish filter ready status
        auto status_msg = std_msgs::msg::Bool();
        status_msg.data = depth_filter_.isReady();
        status_pub_->publish(status_msg);

        if (!depth_filter_.isReady()) {
            RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                "Waiting for filter to be ready... (%zu/%zu samples)",
                depth_filter_.getSampleCount(), static_cast<size_t>(5));
            return;
        }

        // Get filtered distance (using median for robustness)
        double filtered_distance = depth_filter_.getMedianValue();
        
        if (std::isnan(filtered_distance)) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                "No valid filtered distance available");
            return;
        }

        // Publish filtered distance
        auto filtered_msg = std_msgs::msg::Float32();
        filtered_msg.data = filtered_distance;
        filtered_distance_pub_->publish(filtered_msg);

        // Calculate error (positive = too far, need to move closer)
        double error = filtered_distance - target_distance_;

        // Create offset message
        auto offset_msg = geometry_msgs::msg::Vector3Stamped();
        offset_msg.header.stamp = this->now();
        offset_msg.header.frame_id = "base_link";

        // Only apply correction if outside tolerance
        if (std::abs(error) > distance_tolerance_) {
            // Calculate correction with proportional control
            double correction = -kp_ * error;  // Negative: if too far, move closer (negative offset)
            
            // Clamp correction to maximum allowed
            correction = std::clamp(correction, -max_correction_, max_correction_);
            
            // Publish lateral offset (assumes X is forward, Y is lateral)
            offset_msg.vector.x = 0.0;      // No forward/back correction
            offset_msg.vector.y = correction;  // Lateral correction to maintain distance
            offset_msg.vector.z = 0.0;      // No vertical correction

            RCLCPP_DEBUG(this->get_logger(), 
                "Distance: %.3fm (target: %.3fm), Error: %.3fm, Correction: %.3fm",
                filtered_distance, target_distance_, error, correction);
        } else {
            // Within tolerance - no correction needed
            offset_msg.vector.x = 0.0;
            offset_msg.vector.y = 0.0;
            offset_msg.vector.z = 0.0;
        }

        offset_pub_->publish(offset_msg);

        // Log periodic status
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
            "Wall distance: %.3fm (filtered), target: %.3fm, variance: %.4f",
            filtered_distance, target_distance_, depth_filter_.getVariance());
    }

    // Subscriptions
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_image_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr range_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr enable_sub_;

    // Publishers
    rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr offset_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr current_distance_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr filtered_distance_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr status_pub_;

    // Timer
    rclcpp::TimerBase::SharedPtr control_timer_;

    // Filter
    MovingAverageFilter depth_filter_;
    std::mutex filter_mutex_;
    double last_raw_distance_{0.0};

    // Parameters
    double target_distance_;
    double distance_tolerance_;
    double max_correction_;
    double min_valid_distance_;
    double max_valid_distance_;
    bool use_depth_image_;
    double roi_center_x_;
    double roi_center_y_;
    double roi_width_;
    double roi_height_;
    double depth_scale_;
    double kp_;
    bool controller_enabled_;
};

}  // namespace wall_distance_controller

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<wall_distance_controller::WallDistanceControllerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
