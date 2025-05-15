#include <string>
#include <iomanip>
#include <sstream>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/opencv.hpp"


class ImageSaverNode : public rclcpp::Node
{
public:
    ImageSaverNode() : Node("image_saver_node"), image_count_(0), saved_count_(0)
    {
        subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/image_raw", 10,
            std::bind(&ImageSaverNode::image_callback, this, std::placeholders::_1));
        RCLCPP_INFO(this->get_logger(), "ImageSaverNode started, subscribing to '/raw_image'");
    }

private:
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        image_count_++;
        if (image_count_ % 10 != 0)
            return;

        try
        {
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, msg->encoding);
            std::ostringstream filename;
            filename << "/image_data/saved_image_" << std::setw(5) << std::setfill('0') << saved_count_ << ".png";
            if (!cv::imwrite(filename.str(), cv_ptr->image)) {
                RCLCPP_ERROR(this->get_logger(), "Failed to save image: %s", filename.str().c_str());
            }
            RCLCPP_INFO(this->get_logger(), "Saved %s", filename.str().c_str());
            saved_count_++;
        }
        catch (cv_bridge::Exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
    size_t image_count_;
    size_t saved_count_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImageSaverNode>());
    rclcpp::shutdown();
    return 0;
}