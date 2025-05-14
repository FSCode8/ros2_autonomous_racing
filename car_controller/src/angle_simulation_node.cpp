#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include <random>
#include <chrono>
#include <cmath>  // Für std::fabs

using namespace std::chrono_literals;

class AngleSimulationNode : public rclcpp::Node {
public:
  AngleSimulationNode()
  : Node("angle_simulation_node"),
    distribution_(0.0, 90.0),
    random_engine_(std::random_device{}()),
    published_once_(false)
  {
    publisher_ = this->create_publisher<std_msgs::msg::Float64>("target/angle", 10);
    timer_ = this->create_wall_timer(
      10s, std::bind(&AngleSimulationNode::publish_angle, this));
    RCLCPP_INFO(this->get_logger(), "AngleSimulationNode started.");
    RCLCPP_DEBUG(this->get_logger(), "Initialized random engine and distribution.");
  }

private:
  void publish_angle() {
    double new_angle = distribution_(random_engine_);
    // Falls noch nie veröffentlicht oder der neue Winkel weicht vom zuletzt gesendeten signifikant ab.
    if (!published_once_ || std::fabs(new_angle - last_angle_) > 1e-6) {
      last_angle_ = new_angle;
      published_once_ = true;
      auto angle_msg = std_msgs::msg::Float64();
      angle_msg.data = new_angle;
      publisher_->publish(angle_msg);
      RCLCPP_INFO(this->get_logger(), "Published angle: %.2f degrees", new_angle);
    } else {
      RCLCPP_DEBUG(this->get_logger(), "Kein neuer Winkel, veröffentliche nichts.");
    }
  }

  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::uniform_real_distribution<double> distribution_;
  std::mt19937 random_engine_;
  bool published_once_;
  double last_angle_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AngleSimulationNode>());
  rclcpp::shutdown();
  return 0;
}