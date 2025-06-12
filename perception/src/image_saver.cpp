#include <chrono>
#include <functional>
#include <iostream>
#include <memory>
#include <string>
#include <filesystem>
#include <fstream>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/serialization.hpp"
#include "rosbag2_transport/reader_writer_factory.hpp"
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

class ImageSaverNode : public rclcpp::Node
{
    public:
        ImageSaverNode()
        : Node("image_saver_node"), image_count_(0)
        {      
            this->declare_parameter<std::string>("bag_filename", "");
            this->declare_parameter<std::string>("output_folder", "saved_images");

            std::string bag_filename = this->get_parameter("bag_filename").as_string();
            output_folder_ = this->get_parameter("output_folder").as_string();

            if (bag_filename.empty()) {
                RCLCPP_ERROR(this->get_logger(), "No bag filename provided. Use --ros-args -p bag_filename:=<your_bag>");
                throw std::runtime_error("Bag filename not specified");
            }

            // Create output directory
            std::filesystem::create_directories(output_folder_);

            rosbag2_storage::StorageOptions storage_options;
            storage_options.uri = "./" + bag_filename;
            reader_ = rosbag2_transport::ReaderWriterFactory::make_reader(storage_options);
            reader_->open(storage_options);

            RCLCPP_INFO(this->get_logger(), "Opened bag: %s", bag_filename.c_str());
            RCLCPP_INFO(this->get_logger(), "Saving images to: %s", output_folder_.c_str());

            // Process all images immediately
            process_all_images();
        }

    private:
        void process_all_images()
        {
            while (reader_->has_next()) {
                rosbag2_storage::SerializedBagMessageSharedPtr msg = reader_->read_next();

                if (msg->topic_name != "/my_camera/pylon_ros2_camera_node/image_rect") {
                    continue;
                }

                rclcpp::SerializedMessage serialized_msg(*msg->serialized_data);
                sensor_msgs::msg::Image::SharedPtr ros_msg = std::make_shared<sensor_msgs::msg::Image>();

                serialization_.deserialize_message(&serialized_msg, ros_msg.get());

                save_image(ros_msg);
            }

            RCLCPP_INFO(this->get_logger(), "Finished saving %d images", image_count_);
        }

        void save_image(const sensor_msgs::msg::Image::SharedPtr& img_msg)
        {
            try {
                cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::BGR8);
                
                std::string filename = output_folder_ + "/image_" + 
                                                            std::to_string(img_msg->header.stamp.sec) + "_" +
                                                            std::to_string(img_msg->header.stamp.nanosec) + ".png";
                
                cv::imwrite(filename, cv_ptr->image);
                image_count_++;
                
                RCLCPP_INFO(this->get_logger(), "Saved image %d: %s", image_count_, filename.c_str());
            } catch (cv_bridge::Exception& e) {
                RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            }
        }

        std::string output_folder_;
        int image_count_;
        rclcpp::Serialization<sensor_msgs::msg::Image> serialization_;
        std::unique_ptr<rosbag2_cpp::Reader> reader_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    try {
        auto node = std::make_shared<ImageSaverNode>();
        // No need to spin since we process all images in constructor
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }

    return 0;
}
