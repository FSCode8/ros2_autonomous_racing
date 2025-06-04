#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/twist_stamped.hpp"



class AEBNode : public rclcpp::Node {

public:

  AEBNode() : Node("aeb_node"), distance_(10.0) {

    publisher_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(

      "/ackermann_steering_controller/reference", 10);

    timer_ = this->create_wall_timer(

      std::chrono::milliseconds(200),

      std::bind(&AEBNode::timer_callback, this));

  }



private:

  void timer_callback() {

    simulate_distance();



    auto msg = geometry_msgs::msg::TwistStamped();

    msg.twist.linear.x = (distance_ > 1.0) ? 3.0 : 0.0;

    msg.twist.angular.z = 0.0;

    publisher_->publish(msg);



    if (distance_ > 1.0) {

      RCLCPP_INFO(this->get_logger(), "Abstand: %.2f m – Fahren", distance_);

    } else {

      RCLCPP_WARN(this->get_logger(), "Abstand: %.2f m – Bremsen!", distance_);

    }

  }



  // Simulation der Abstandverringerung

  void simulate_distance() {

    if (distance_ > 0.0) {

      distance_ -= 0.2;

      if (distance_ < 0.0) distance_ = 0.0;

    }

  }



  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr publisher_;

  rclcpp::TimerBase::SharedPtr timer_;

  double distance_;

};



int main(int argc, char **argv) {

  rclcpp::init(argc, argv);

  rclcpp::spin(std::make_shared<AEBNode>());

  rclcpp::shutdown();

  return 0;

}

