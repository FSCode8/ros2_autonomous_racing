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
    client_ = rclcpp_action::create_client<FollowPath>(this, "/follow_path");

    // Wait for the action server
    if (!client_->wait_for_action_server(5s)) {
      RCLCPP_ERROR(get_logger(), "Action server not available!");
      return;
    }

    // Create a dummy path
    auto path_msg = nav_msgs::msg::Path();
    path_msg.header.frame_id = "base_link";
    path_msg.header.stamp = now();

    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = "base_link";
    pose.pose.position.x = 2.0;
    pose.pose.orientation.w = 1.0;
    path_msg.poses.push_back(pose);

    // Create a goal
    auto goal_msg = FollowPath::Goal();
    goal_msg.path = path_msg;

    // Send the goal
    auto send_goal_options = rclcpp_action::Client<FollowPath>::SendGoalOptions();
    send_goal_options.result_callback = [](const GoalHandleFollowPath::WrappedResult & result) {
      RCLCPP_INFO(rclcpp::get_logger("FollowPathClient"), "Result received: %d", static_cast<int>(result.code));
    };

    client_->async_send_goal(goal_msg, send_goal_options);
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