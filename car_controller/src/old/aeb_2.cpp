#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/twist_stamped.hpp"
#include "std_msgs/msg/float32.hpp"



class AEBNode : public rclcpp::Node {

public:

  AEBNode() : Node("aeb_node"), distance_(10.0){
    publisher_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("/ackermann_steering_controller/reference", 10);
    ttc_subscription = this->create_subscription<std_msgs::msg::Float32>("/time_to_collision", 10, std::bind(&AEBNode::ttc_callback, this, std::placeholders::_1));;
    timer_ = this->create_wall_timer(std::chrono::milliseconds(200), std::bind(&AEBNode::timer_callback, this));
  }

private:

  void timer_callback(){
    auto msg = geometry_msgs::msg::TwistStamped();
    msg.twist.linear.x = 9.0;
    if((ttc > 0 && ttc < 2.0))
      msg.twist.linear.x = 0.0;
    msg.twist.angular.z = 0.0;
    publisher_->publish(msg);
  }
  
  void ttc_callback(const std_msgs::msg::Float32 & msg){
    ttc = msg.data;
  }

  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr publisher_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr ttc_subscription;
  rclcpp::TimerBase::SharedPtr timer_;
  double distance_;
  float ttc;
};


int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AEBNode>());
  rclcpp::shutdown();
  return 0;
}
