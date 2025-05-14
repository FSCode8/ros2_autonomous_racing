#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "std_msgs/msg/float64.hpp"
#include <cmath>

class BicycleControlNode : public rclcpp::Node {
public:
  BicycleControlNode() : Node("bicycle_control_node"), remaining_heading_change_(0.0) {
    // Parameter initialisieren, damit die Geschwindigkeit dynamisch änderbar ist
    this->declare_parameter<double>("linear_speed", 1.5);

    angle_subscriber_ = this->create_subscription<std_msgs::msg::Float64>(
      "target/angle", 10,
      std::bind(&BicycleControlNode::angle_callback, this, std::placeholders::_1));

    twist_publisher_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
      "/ackermann_steering_controller/reference", 10);

    // Initialisiere last_time_ für eine dynamische dt-Berechnung
    last_time_ = this->now();

    // Control loop timer (100ms Intervall, dt wird aber dynamisch berechnet)
    control_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&BicycleControlNode::control_loop, this));

    RCLCPP_INFO(this->get_logger(), "BicycleControlNode started.");
  }

private:
  void angle_callback(const std_msgs::msg::Float64::SharedPtr msg) {
    remaining_heading_change_ += msg->data;
    RCLCPP_INFO(this->get_logger(),
      "Received new relative heading request: %.2f deg. Total remaining change: %.2f deg",
      msg->data, remaining_heading_change_);
  }

  void control_loop() {
    // Dynamisch dt berechnen (vergangene Zeit in Sekunden seit dem letzten Aufruf)
    auto current_time = this->now();
    double dt = (current_time - last_time_).seconds();
    last_time_ = current_time;

    double k_p = 0.5;                // proportional gain [deg/s per deg]
    double max_turn_rate = 10.0;       // maximale Drehrate in deg/s
    double min_turn_rate = 0.1;        // minimale effektive Drehrate in deg/s

    double angular_command = 0.0;
    if (std::abs(remaining_heading_change_) > 0.5) { // Steuern nur, wenn der Fehler signifikant ist
      angular_command = k_p * remaining_heading_change_;

      // Sättigung der Drehgeschwindigkeit
      if (angular_command > max_turn_rate) {
        angular_command = max_turn_rate;
      } else if (angular_command < -max_turn_rate) {
        angular_command = -max_turn_rate;
      }
      // Sicherstellen, dass ein Mindestwert überschritten wird
      if (std::abs(angular_command) < min_turn_rate) {
        angular_command = (angular_command < 0) ? -min_turn_rate : min_turn_rate;
      }
    } else {
      angular_command = 0.0;
    }

    // Simuliere das Abfahren des Winkels: Integration der Drehgeschwindigkeit
    double executed_change = angular_command * dt;
    if (std::abs(executed_change) > std::abs(remaining_heading_change_)) {
      executed_change = remaining_heading_change_;
    }
    remaining_heading_change_ -= executed_change;

    // Dynamisch gelesene lineare Geschwindigkeit
    double linear_speed = this->get_parameter("linear_speed").as_double();

    auto twist_msg = geometry_msgs::msg::TwistStamped();
    twist_msg.twist.linear.x = linear_speed;
    twist_msg.twist.angular.z = angular_command;
    twist_publisher_->publish(twist_msg);

    RCLCPP_DEBUG(this->get_logger(),
      "dt: %.2f sec, angular command: %.2f deg/s, executed: %.2f deg, remaining change: %.2f deg",
      dt, angular_command, executed_change, remaining_heading_change_);
  }

  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr angle_subscriber_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_publisher_;
  rclcpp::TimerBase::SharedPtr control_timer_;
  
  rclcpp::Time last_time_;
  double remaining_heading_change_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BicycleControlNode>());
  rclcpp::shutdown();
  return 0;
}