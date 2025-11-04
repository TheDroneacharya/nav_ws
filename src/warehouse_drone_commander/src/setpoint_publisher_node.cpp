#include <rclcpp/rclcpp.hpp>
#include <mavros_msgs/msg/state.hpp>
#include <mavros_msgs/msg/position_target.hpp>
#include <mavros_msgs/srv/set_mode.hpp>
#include <mavros_msgs/srv/command_bool.hpp>

#include <chrono>

using namespace std::placeholders;
using namespace std::chrono_literals;

class SetpointPublisher : public rclcpp::Node
{
public:
    SetpointPublisher() : Node("setpoint_publisher")
    {
        // Initialization code here
        RCLCPP_INFO(this->get_logger(), "Starting setpoint publisher node...");

        setpoint_sub_ = this->create_subscription<mavros_msgs::msg::PositionTarget>("offboard_commander/setpoint_raw/local", 10, std::bind(&SetpointPublisher::setpoint_update_callback, this, _1));
        setpoint_pub_ = this->create_publisher<mavros_msgs::msg::PositionTarget>("mavros/setpoint_raw/local", 10);

        // TODO: declare parameters for timers
        setpoint_publish_timer_ = this->create_wall_timer(100ms, setpoint_publish_callback);    // 100ms = 10 Hz timer
    }

private:

    void setpoint_update_callback(const mavros_msgs::msg::PositionTarget msg){
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

        if (!setpoint_received_) {
            RCLCPP_INFO(this->get_logger(), "First setpoint target received from commander.");
            setpoint_received_ = true;
        }
    }

    void setpoint_publish_callback(){
        // Wait for setpoint from commander
        if (setpoint_received_) {
            // Set appropriate header timestamp
            setpoint_to_send_.header.stamp = this->now();
            // Publish setpoint
            setpoint_pub_->publish(setpoint_to_send_);
        }
    }

    rclcpp::Subscription<mavros_msgs::msg::PositionTarget>::SharedPtr setpoint_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr setpoint_pub_;

    rclcpp::TimerBase::SharedPtr setpoint_publish_timer_;

    bool setpoint_received_ = false;

    mavros_msgs::msg::PositionTarget setpoint_to_send_;
};

int main(int argc, char *argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<SetpointPublisher>());
	rclcpp::shutdown();
	return 0;
}
