#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"

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
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr vel_subscription;
    size_t count;
    float ttc, distance, velocity;

    //methods
    void ttc_callback();
    void distance_callback(const std_msgs::msg::Float32 & msg);
    void velocity_callback(const geometry_msgs::msg::TwistStamped & msg);
};