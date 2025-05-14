#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include <chrono>
#include <cmath>

#ifndef M_PI
#define M_PI (3.14159265358979323846)
#endif

using namespace std::chrono_literals;

class AngleSimulationNode : public rclcpp::Node {
public:
  AngleSimulationNode()
  : Node("angle_simulation_node"), current_angle_(-0.785), published_once_(false)
  {
    publisher_ = this->create_publisher<std_msgs::msg::Float64>("target/angle", 10);
    start_time_ = this->now();
    // Timer: alle 10 s wird geprüft, ob ein neuer Winkel vorhanden ist
    timer_ = this->create_wall_timer(
      10s, std::bind(&AngleSimulationNode::publish_angle, this));
    RCLCPP_INFO(this->get_logger(), "AngleSimulationNode gestartet mit initial -0.785 rad (ca. -45°).");
  }

private:
  void publish_angle() {
    // Nach 20 s soll auf 0.785 rad gewechselt werden (einmaliger Wechsel)
    if ((this->now() - start_time_).seconds() >= 30.0 && current_angle_ != 0.785) {
      current_angle_ = 0.785;
      RCLCPP_INFO(this->get_logger(), "30 Sekunden vergangen. Schalte auf 0.785 rad (ca. 45°) um.");
    }
    // Nur veröffentlichen, wenn sich der aktuelle Winkel ändert
    if (!published_once_ || std::fabs(current_angle_ - last_published_angle_) > 1e-6) {
      auto angle_msg = std_msgs::msg::Float64();
      angle_msg.data = current_angle_;
      publisher_->publish(angle_msg);
      RCLCPP_INFO(this->get_logger(), "Veröffentliche Winkel: %.4f rad (ca. %.1f°)", 
                  angle_msg.data, angle_msg.data * 180.0 / M_PI);
      last_published_angle_ = current_angle_;
      published_once_ = true;
    } else {
      RCLCPP_DEBUG(this->get_logger(), "Kein neuer Winkel, veröffentliche nichts.");
    }
  }

  rclcpp::Time start_time_;
  double current_angle_;
  double last_published_angle_;
  bool published_once_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AngleSimulationNode>());
  rclcpp::shutdown();
  return 0;
}