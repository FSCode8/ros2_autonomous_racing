#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"

class TTC_Node : public rclcpp::Node
{
  public:
    //constructor
    TTC_Node();

  private:
    //variables
    rclcpp::TimerBase::SharedPtr timer;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr ttc_publisher;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr dist_subscription;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr vel_subscription;
    size_t count;
    float ttc, distance, velocity;

    //methods
    void ttc_callback();
    void distance_callback(const std_msgs::msg::Float32 & msg);
    void velocity_callback(const std_msgs::msg::Float32 & msg);
};