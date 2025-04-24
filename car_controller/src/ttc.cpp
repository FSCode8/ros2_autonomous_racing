#include "../include/car_controller/ttc.h"

using namespace std::chrono_literals;

TTC_Node::TTC_Node() : Node("ttc_node"), count(0){
  ttc_publisher = this->create_publisher<std_msgs::msg::Float32>("/time_to_collision", 10);
  dist_subscription = this->create_subscription<std_msgs::msg::Float32>("/collision_distance", 10, std::bind(&TTC_Node::distance_callback, this, std::placeholders::_1));
  vel_subscription = this->create_subscription<geometry_msgs::msg::TwistStamped>("/ackermann_steering_controller/reference", 10, std::bind(&TTC_Node::velocity_callback, this, std::placeholders::_1));
  timer = this->create_wall_timer(33ms, std::bind(&TTC_Node::ttc_callback, this));
}

void TTC_Node::ttc_callback(){
  std_msgs::msg::Float32 ttc_msg;
  ttc_msg.data = ttc;
  ttc = distance / velocity;
  ttc_publisher->publish(ttc_msg);
}

void TTC_Node::distance_callback(const std_msgs::msg::Float32 & msg){
  distance = msg.data;
}

void TTC_Node::velocity_callback(const geometry_msgs::msg::TwistStamped & msg){
  velocity = msg.twist.linear.x;  // Figure out which parameter in message holds velocity
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TTC_Node>());
  rclcpp::shutdown();
  return 0;
}