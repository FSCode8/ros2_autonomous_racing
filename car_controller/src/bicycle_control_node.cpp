#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "std_msgs/msg/float64.hpp"
#include <cmath>

class BicycleControlNode : public rclcpp::Node {
public:
  BicycleControlNode() : Node("bicycle_control_node"), remaining_heading_change_(0.0) {
    angle_subscriber_ = this->create_subscription<std_msgs::msg::Float64>(
      "target/angle", 10,
      std::bind(&BicycleControlNode::angle_callback, this, std::placeholders::_1));

    twist_publisher_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
      "/ackermann_steering_controller/reference", 10);

    // Control loop timer (dt = 0.1 sec)
    control_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&BicycleControlNode::control_loop, this));

    RCLCPP_INFO(this->get_logger(), "BicycleControlNode started.");
  }

private:
  // When a new target angle is received (as a relative heading change in degrees)
  void angle_callback(const std_msgs::msg::Float64::SharedPtr msg) {
    // Accumulate the desired heading change.
    remaining_heading_change_ += msg->data;
    RCLCPP_INFO(this->get_logger(),
      "Received new relative heading request: %.2f deg. Total remaining change: %.2f deg",
      msg->data, remaining_heading_change_);
  }

  // Simulated control loop: determine steering command based on the remaining required heading change.
  void control_loop() {
    double dt = 0.1;  // time step in sec (100ms)
    double k_p = 0.5; // proportional gain [deg/s per deg]
    double max_turn_rate = 10.0; // maximum turn rate in deg/s
    double min_turn_rate = 0.1;  // minimum effective turn rate in deg/s

    double angular_command = 0.0;
    if (std::abs(remaining_heading_change_) > 0.5) { // Only command turning if error is significant
      angular_command = k_p * remaining_heading_change_;

      // Saturate the command to avoid unrealistic turning rates
      if (angular_command > max_turn_rate) {
        angular_command = max_turn_rate;
      } else if (angular_command < -max_turn_rate) {
        angular_command = -max_turn_rate;
      }
      // Ensure a minimum command is sent so that a small error isn't ignored
      if (std::abs(angular_command) < min_turn_rate) {
        angular_command = (angular_command < 0) ? -min_turn_rate : min_turn_rate;
      }
    } else {
      angular_command = 0.0;
    }

    // Simulate turning: integrate the command over time.
    // This approximates how much of the requested heading change is executed.
    double executed_change = angular_command * dt;
    // Avoid overshooting: if executed change is larger than the remaining, cap it.
    if (std::abs(executed_change) > std::abs(remaining_heading_change_)) {
      executed_change = remaining_heading_change_;
    }
    remaining_heading_change_ -= executed_change;

    auto twist_msg = geometry_msgs::msg::TwistStamped();
    twist_msg.twist.linear.x = 1.5;            // Constant speed
    twist_msg.twist.angular.z = angular_command; // Steering command in deg/s (simulated)
    twist_publisher_->publish(twist_msg);

    RCLCPP_DEBUG(this->get_logger(),
      "dt: %.2f sec, angular command: %.2f deg/s, executed: %.2f deg, remaining change: %.2f deg",
      dt, angular_command, executed_change, remaining_heading_change_);
  }

  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr angle_subscriber_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_publisher_;
  rclcpp::TimerBase::SharedPtr control_timer_;

  double remaining_heading_change_; // in degrees, remaining desired heading change
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BicycleControlNode>());
  rclcpp::shutdown();
  return 0;
}