#include <rclcpp/rclcpp.hpp>
#include <rosbag2_cpp/writer.hpp>

#include "sensor_msgs/msg/image.hpp"

using std::placeholders::_1;

class BagRecorder : public rclcpp::Node
{
public:
  BagRecorder(const std::string & bag_name)
  : Node("recorder_node")
  {
    writer_ = std::make_unique<rosbag2_cpp::Writer>();

    writer_->open("src/"+ bag_name);

    subscription_ = create_subscription<sensor_msgs::msg::Image>(
      "/image_raw", 10, std::bind(&BagRecorder::topic_callback, this, _1));
  }

private:
  void topic_callback(std::shared_ptr<rclcpp::SerializedMessage> msg) const
  {
    rclcpp::Time time_stamp = this->now();

    writer_->write(msg, "/image_raw", "sensor_msgs/msg/Image", time_stamp);
  }

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
  std::unique_ptr<rosbag2_cpp::Writer> writer_;
};

int main(int argc, char** argv)
{
  if (argc != 2) {
    std::cerr << "Usage: " << argv[0] << " <bag>" << std::endl;
    return 1;
  }

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BagRecorder>(argv[1]));
  rclcpp::shutdown();
  return 0;
}
