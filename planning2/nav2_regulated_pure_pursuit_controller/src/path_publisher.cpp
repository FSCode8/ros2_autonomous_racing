#include <memory>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "nav2_msgs/action/follow_path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"

using namespace std::chrono_literals;

class FollowPathClient : public rclcpp::Node
{
public:
  using FollowPath = nav2_msgs::action::FollowPath;
  using GoalHandleFollowPath = rclcpp_action::ClientGoalHandle<FollowPath>;

  FollowPathClient()
  : Node("follow_path_cpp_client")
  {
    this->client_ = rclcpp_action::create_client<FollowPath>(this, "/follow_path");

    // Wait for the action server
    if (!this->client_->wait_for_action_server(5s)) {
      RCLCPP_ERROR(get_logger(), "Action server not available!");
      return;
    } else {
      RCLCPP_INFO(get_logger(), "Action server available!");
    }

    // Create a longer path
    auto path_msg = nav_msgs::msg::Path();
    path_msg.header.frame_id = "odom";
    path_msg.header.stamp = now();

    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = "odom";
    pose.pose.orientation.w = 1.0;

    // Add 20 points spaced 0.5m apart along the x-axis
    for (int i = 0; i < 20; ++i) {
      pose.pose.position.x = i * 0.5;
      pose.pose.position.y = 0.0;
      path_msg.poses.push_back(pose);
    }

    // Create a goal
    auto goal_msg = FollowPath::Goal();
    goal_msg.path = path_msg;

    // Send the goal
    auto send_goal_options = rclcpp_action::Client<FollowPath>::SendGoalOptions();
    send_goal_options.result_callback = [](const GoalHandleFollowPath::WrappedResult & result) {
      RCLCPP_INFO(rclcpp::get_logger("FollowPathClient"), "Result received: %d", static_cast<int>(result.code));
    };

    this->client_->async_send_goal(goal_msg, send_goal_options);
  }

private:
  rclcpp_action::Client<FollowPath>::SharedPtr client_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<FollowPathClient>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}