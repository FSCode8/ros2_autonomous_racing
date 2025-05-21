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

using std::placeholders::_1;

class TransformNode : public rclcpp::Node
{
  public:
    TransformNode()
    : Node("playback_node")
    {
      subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
        "/image_raw", 10, std::bind(&TransformNode::read_image, this, _1));
    }

  private:
    void read_image(const sensor_msgs::msg::Image & msg) const
    {
        //sensor_msgs::msg::Image::SharedPtr ros_msg = std::make_shared<sensor_msgs::msg::Image>(msg);

        try {
          /* Threshold type
          0: Binary
          1: Binary Inverted
          2: Threshold Truncated
          3: Threshold to Zero
          4: Threshold to Zero Inverted
          */
          int threshold_type = 3; 
          int threshold_value = 190; // max 255
          int const max_binary_value = 255;

          // Convert ROS Image message to OpenCV image
          cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
          cv::Mat image = cv_ptr->image;
          
          // Convert to grayscale
          cv::Mat img_gray, img_thresh;
          cv::cvtColor(image, img_gray, cv::COLOR_BGR2GRAY); 
          
          // Display the image in a window
          cv::imshow("Image Viewer", img_gray);
          cv::waitKey(0);

          cv::threshold(img_gray, img_thresh, threshold_value, max_binary_value, threshold_type);

          // Display the image in a window
          cv::imshow("Image Viewer", img_thresh);
          cv::waitKey(0);

        } catch (const cv_bridge::Exception& e) {
          std::cerr << "cv_bridge exception: " << e.what() << std::endl;
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin_some(std::make_shared<TransformNode>());
  rclcpp::shutdown();

  return 0;
}
