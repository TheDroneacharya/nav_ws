#include <memory>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"

class PlanPublisher : public rclcpp::Node
{
public:
	PlanPublisher()
	: Node("plan_publisher")
	{
		this->declare_parameter<std::string>("plan_in", "/plan");
		this->declare_parameter<std::string>("plan_out", "/plan_viz");

		this->get_parameter("plan_in", plan_in_);
		this->get_parameter("plan_out", plan_out_);

		publisher_ = this->create_publisher<nav_msgs::msg::Path>(plan_out_, rclcpp::QoS(10));

		subscription_ = this->create_subscription<nav_msgs::msg::Path>(
			plan_in_, rclcpp::QoS(10),
			std::bind(&PlanPublisher::topic_callback, this, std::placeholders::_1));
		RCLCPP_INFO(this->get_logger(), "Subscribed to '%s' and publishing fixed paths to '%s'", plan_in_.c_str(), plan_out_.c_str());
	}

private:
	void topic_callback(const nav_msgs::msg::Path::SharedPtr msg)
	{
		if (!msg) {
			RCLCPP_WARN(this->get_logger(), "Received null Path message");
			return;
		}

		nav_msgs::msg::Path out = *msg; // copy so we can modify

		// If the Path header is empty, we can't fix PoseStamped headers reliably.
		const bool path_header_empty = out.header.frame_id.empty() && out.header.stamp.sec == 0 && out.header.stamp.nanosec == 0;
		if (path_header_empty) {
			RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
								 "Path message header is empty; not modifying pose headers");
			publisher_->publish(out);
			return;
		}

		bool modified = false;
		for (auto &pose_stamped : out.poses)
		{
			const bool pose_header_empty = pose_stamped.header.frame_id.empty() &&
										   pose_stamped.header.stamp.sec == 0 &&
										   pose_stamped.header.stamp.nanosec == 0;
			if (pose_header_empty)
			{
				pose_stamped.header.frame_id = out.header.frame_id;
				pose_stamped.header.stamp = out.header.stamp;
				modified = true;
			}
		}


		if (modified)
		{
			RCLCPP_DEBUG(this->get_logger(), "Fixed %zu empty PoseStamped headers", out.poses.size());
		}

		publisher_->publish(out);
	}

	rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr publisher_;
	rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr subscription_;
	std::string plan_in_;
	std::string plan_out_;
};

int main(int argc, char *argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<PlanPublisher>());
	rclcpp::shutdown();
	return 0;
}
