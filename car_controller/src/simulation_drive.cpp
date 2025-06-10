#include <chrono>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>

using namespace std::chrono_literals;

class BicycleControllerNode : public rclcpp::Node
{
public:
    BicycleControllerNode()
        : Node("bicycle_controller_node"), state_(0)
    {
        publisher_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("platzhalter/reference", 10);
        timer_ = this->create_wall_timer(500ms, std::bind(&BicycleControllerNode::timerCallback, this));
        start_time_ = this->now();
    }

private:
    void timerCallback()
    {
        auto current_time = this->now();
        auto elapsed_time = (current_time - start_time_).seconds();

        // Update state every 5 seconds
        if (elapsed_time >= 5.0) {
            state_ = (state_ + 1) % 3; // Cycle through 0 (straight), 1 (right), 2 (left)
            start_time_ = current_time;
        }

        // Create and publish TwistStamped message
        auto twist_msg = geometry_msgs::msg::TwistStamped();
        twist_msg.header.stamp = this->now();
        twist_msg.header.frame_id = "base_link";

        if (state_ == 0) {
            // Straight
            twist_msg.twist.linear.x = 0.22;  // Move forward
            twist_msg.twist.angular.z = 0.0; // No rotation
        } else if (state_ == 1) {
            // Turn right
            twist_msg.twist.linear.x = 0.3;  // Move forward
            twist_msg.twist.angular.z = -1.0; // Turn right
        } else if (state_ == 2) {
            // Turn left
            twist_msg.twist.linear.x = 0.3;  // Move forward
            twist_msg.twist.angular.z = 1.0; // Turn left
        }

        publisher_->publish(twist_msg);
    }

    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Time start_time_;
    int state_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BicycleControllerNode>());
    rclcpp::shutdown();
    return 0;
}