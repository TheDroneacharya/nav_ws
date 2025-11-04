#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

class CmdVelPublisher : public rclcpp::Node
{
public:
	CmdVelPublisher() : Node("cmd_vel_publisher")
	{
		this->declare_parameter<std::string>("cmd_vel_in","cmd_vel");

		this->get_parameter("cmd_vel_in", cmd_vel_in);

		publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel_enu", 1);

		subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(cmd_vel_in, 10, std::bind(&CmdVelPublisher::topic_callback, this, std::placeholders::_1));
	}

private:
	void topic_callback(const geometry_msgs::msg::Twist & twist_in) const
	{		
		geometry_msgs::msg::Twist twist_out;
		twist_out.linear.x = -twist_in.linear.y;	// FLU left becomes ENU East
		twist_out.linear.y = twist_in.linear.x;		// FLU forward becomes ENU North
		twist_out.linear.z = twist_in.linear.z;		// FLU up is ENU up

		twist_out.angular.x = twist_in.angular.x;	// FLU roll is ENU roll if Z is Up
		twist_out.angular.y = twist_in.angular.y;	// FLU pitch is ENU pitch if Z is Up
		twist_out.angular.z = -twist_in.angular.z;	// FLU yaw (CCW positive) becomes ENU yaw (CW positive), flipped because of the axis swap

		if (publisher_)
		{
			publisher_->publish(twist_out);
		}
		else
		{
			RCLCPP_ERROR(this->get_logger(), "Publisher is uninitialized!");
		}
	}
	rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
	rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;
	std::string cmd_vel_in;
};

int main(int argc, char *argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<CmdVelPublisher>());
	rclcpp::shutdown();
	return 0;
}
