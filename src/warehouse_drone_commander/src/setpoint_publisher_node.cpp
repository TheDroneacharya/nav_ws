#include <chrono>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <mavros_msgs/msg/state.hpp>
#include <mavros_msgs/msg/position_target.hpp>
#include <mavros_msgs/srv/set_mode.hpp>
#include <mavros_msgs/srv/command_bool.hpp>

using namespace std::placeholders;
using namespace std::chrono_literals;

class SetpointPublisher : public rclcpp::Node
{
public:
    SetpointPublisher(const rclcpp::NodeOptions& options) : Node("setpoint_publisher", options)
    {
        // Initialization code here
        RCLCPP_INFO(this->get_logger(), "Setpoint publisher node constructed...");

        auto sensor_qos = rclcpp::SensorDataQoS();

        setpoint_sub_commander = this->create_subscription<mavros_msgs::msg::PositionTarget>("offboard_commander/setpoint_raw/local", sensor_qos, std::bind(&SetpointPublisher::setpoint_update_commander_callback, this, _1));
        setpoint_sub_action_server = this->create_subscription<mavros_msgs::msg::PositionTarget>("go_to_pose_action_server/setpoint_raw/local", sensor_qos, std::bind(&SetpointPublisher::setpoint_update_action_server_callback, this, _1));
        setpoint_pub_ = this->create_publisher<mavros_msgs::msg::PositionTarget>("mavros/setpoint_raw/local", 10);

        // TODO: declare parameters for timers
        setpoint_publish_timer_ = this->create_wall_timer(100ms, std::bind(&SetpointPublisher::setpoint_publish_callback, this));    // 100ms = 10 Hz timer
    }

private:

    void setpoint_update_commander_callback(const mavros_msgs::msg::PositionTarget msg){
        RCLCPP_WARN(this->get_logger(), "Received new setpoint target from COMMANDER.");

        setpoint_to_send_.header.frame_id = msg.header.frame_id;
        setpoint_to_send_.coordinate_frame = msg.coordinate_frame;
        setpoint_to_send_.type_mask = msg.type_mask;
        setpoint_to_send_.position.x = msg.position.x;
        setpoint_to_send_.position.y = msg.position.y;
        setpoint_to_send_.position.z = msg.position.z;
        setpoint_to_send_.velocity.x = msg.velocity.x;
        setpoint_to_send_.velocity.y = msg.velocity.y;
        setpoint_to_send_.velocity.z = msg.velocity.z;
        setpoint_to_send_.acceleration_or_force.x = msg.acceleration_or_force.x;
        setpoint_to_send_.acceleration_or_force.y = msg.acceleration_or_force.y;
        setpoint_to_send_.acceleration_or_force.z = msg.acceleration_or_force.z;
        setpoint_to_send_.yaw = msg.yaw;
        setpoint_to_send_.yaw_rate = msg.yaw_rate;

        if (!setpoint_received_commander_) {
            RCLCPP_WARN(this->get_logger(), "First setpoint target received from COMMANDER.");
            setpoint_received_commander_ = true;
        }
    }

    void setpoint_update_action_server_callback(const mavros_msgs::msg::PositionTarget msg){
        RCLCPP_WARN(this->get_logger(), "Received new setpoint target from ACTION SERVER.");

        setpoint_to_send_.header.frame_id = msg.header.frame_id;
        setpoint_to_send_.coordinate_frame = msg.coordinate_frame;
        setpoint_to_send_.type_mask = msg.type_mask;
        setpoint_to_send_.position.x = msg.position.x;
        setpoint_to_send_.position.y = msg.position.y;
        setpoint_to_send_.position.z = msg.position.z;
        setpoint_to_send_.velocity.x = msg.velocity.x;
        setpoint_to_send_.velocity.y = msg.velocity.y;
        setpoint_to_send_.velocity.z = msg.velocity.z;
        setpoint_to_send_.acceleration_or_force.x = msg.acceleration_or_force.x;
        setpoint_to_send_.acceleration_or_force.y = msg.acceleration_or_force.y;
        setpoint_to_send_.acceleration_or_force.z = msg.acceleration_or_force.z;
        setpoint_to_send_.yaw = msg.yaw;
        setpoint_to_send_.yaw_rate = msg.yaw_rate;

        if (!setpoint_received_action_server_) {
            RCLCPP_WARN(this->get_logger(), "First setpoint target received from ACTION SERVER.");
            setpoint_received_action_server_ = true;
        }
    }

    void setpoint_publish_callback(){
        // Wait for setpoint from commander
        if (setpoint_received_commander_ || setpoint_received_action_server_) {
            // Set appropriate header timestamp
            setpoint_to_send_.header.stamp = this->now();
            // Publish setpoint
            setpoint_pub_->publish(setpoint_to_send_);
        }
    }

    rclcpp::Subscription<mavros_msgs::msg::PositionTarget>::SharedPtr setpoint_sub_commander;
    rclcpp::Subscription<mavros_msgs::msg::PositionTarget>::SharedPtr setpoint_sub_action_server;
    rclcpp::Publisher<mavros_msgs::msg::PositionTarget>::SharedPtr setpoint_pub_;

    rclcpp::TimerBase::SharedPtr setpoint_publish_timer_;

    bool setpoint_received_commander_{false};
    bool setpoint_received_action_server_{false};

    mavros_msgs::msg::PositionTarget setpoint_to_send_;
};

int main(int argc, char *argv[])
{
	rclcpp::init(argc, argv);

    rclcpp::NodeOptions node_options;

    auto setpoint_publisher_node = std::make_shared<SetpointPublisher>(node_options);

    RCLCPP_INFO(setpoint_publisher_node->get_logger(), "Starting Setpoint Publisher Node...");
	rclcpp::spin(setpoint_publisher_node);
    RCLCPP_INFO(setpoint_publisher_node->get_logger(), "Shutting down Setpoint Publisher Node...");

	rclcpp::shutdown();
	return 0;
}
