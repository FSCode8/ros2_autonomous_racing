#include <chrono>
#include <functional>
#include <iostream>
#include <memory>
#include <string>

#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h> 

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/serialization.hpp"
#include "rosbag2_transport/reader_writer_factory.hpp"
#include <sensor_msgs/msg/image.hpp>

using namespace std::chrono_literals;

class PlaybackNode : public rclcpp::Node
{
  public:
    PlaybackNode(const std::string & bag_filename)
    : Node("playback_node")
    {
      publisher_ = this->create_publisher<sensor_msgs::msg::Image>("/image_raw", 10);
      /*
      timer_ = this->create_wall_timer(
          100ms, std::bind(&PlaybackNode::timer_callback, this));
      */
      rosbag2_storage::StorageOptions storage_options;
      storage_options.uri = "src/" + bag_filename;
      reader_ = rosbag2_transport::ReaderWriterFactory::make_reader(storage_options);
      reader_->open(storage_options);
      timer_callback();
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
        //std::cout << '(' << ros_msg->height << ", " << ros_msg->width << ")\n";

        try {
          // Convert ROS Image message to OpenCV image
          cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(ros_msg, sensor_msgs::image_encodings::BGR8);
          cv::Mat image = cv_ptr->image;

          // Display the image in a window
          cv::imshow("Image Viewer", image);
          std::cout << "Image published and displayed." << std::endl;

          // Wait for a key press before closing the window
          cv::waitKey(0);
        } catch (const cv_bridge::Exception& e) {
          std::cerr << "cv_bridge exception: " << e.what() << std::endl;
        }

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
  if (argc != 2) {
    std::cerr << "Usage: " << argv[0] << " <bag>" << std::endl;
    return 1;
  }

  rclcpp::init(argc, argv);
  rclcpp::spin_some(std::make_shared<PlaybackNode>(argv[1]));
  rclcpp::shutdown();

  return 0;
}
