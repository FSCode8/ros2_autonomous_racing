#include <chrono>
#include <functional>
#include <iostream>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/serialization.hpp"
#include "rosbag2_transport/reader_writer_factory.hpp"
#include <sensor_msgs/msg/image.hpp>

using namespace std::chrono_literals;

class PlaybackNode : public rclcpp::Node
{
  public:
    PlaybackNode()
    : Node("playback_node")
    {      
      this->declare_parameter<std::string>("bag_filename", ""); // Declare and get parameter for bag filename

      std::string bag_filename = this->get_parameter("bag_filename").as_string();

      if (bag_filename.empty()) {
        RCLCPP_ERROR(this->get_logger(), "No bag filename provided. Use --ros-args -p bag_filename:=<your_bag>");
        throw std::runtime_error("Bag filename not specified");
      }

      publisher_ = this->create_publisher<sensor_msgs::msg::Image>("/image_raw", 10);
      timer_ = this->create_wall_timer(
          100ms, std::bind(&PlaybackNode::timer_callback, this));

      rosbag2_storage::StorageOptions storage_options;
      storage_options.uri = "src/" + bag_filename;
      reader_ = rosbag2_transport::ReaderWriterFactory::make_reader(storage_options);
      reader_->open(storage_options);

      RCLCPP_INFO(this->get_logger(), "Opened bag: %s", bag_filename.c_str());
    }

  private:
    void timer_callback()
    {
      while (reader_->has_next()) {
        rosbag2_storage::SerializedBagMessageSharedPtr msg = reader_->read_next();

        if (msg->topic_name != "/image_raw") {
          continue;
        }

        rclcpp::SerializedMessage serialized_msg(*msg->serialized_data);
        sensor_msgs::msg::Image::SharedPtr ros_msg = std::make_shared<sensor_msgs::msg::Image>();

        serialization_.deserialize_message(&serialized_msg, ros_msg.get());

        publisher_->publish(*ros_msg);
        RCLCPP_INFO(this->get_logger(), "Image published: %d.%d", ros_msg->header.stamp.sec, ros_msg->header.stamp.nanosec);

        break;
      }
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;

    rclcpp::Serialization<sensor_msgs::msg::Image> serialization_;
    std::unique_ptr<rosbag2_cpp::Reader> reader_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  try {
    auto node = std::make_shared<PlaybackNode>();
    rclcpp::spin(node);
  } catch (const std::exception& e) {
    std::cerr << "Error: " << e.what() << std::endl;
    return 1;
  }

  return 0;
}
