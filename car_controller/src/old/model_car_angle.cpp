#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "std_msgs/msg/float64.hpp"
#include <cmath>

template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}

class AngleNode : public rclcpp::Node {
public:
    enum class DriveMode {
        STOPPED,
        TURNING,
        STRAIGHT
    };

    AngleNode() : Node("angle"), remaining_heading_change_(0.0), drive_mode_(DriveMode::STRAIGHT) {
        if (!this->has_parameter("use_sim_time")) {
            this->declare_parameter<bool>("use_sim_time", false);
            RCLCPP_INFO(this->get_logger(), "use_sim_time nicht gesetzt, nutze default (false)");
        } else {
            RCLCPP_INFO(this->get_logger(), "use_sim_time bereits vorhanden, nutze existierenden Wert");
        }

        bool use_sim_time = this->get_parameter("use_sim_time").as_bool();

        if (use_sim_time) {
            RCLCPP_INFO(this->get_logger(), "Nutze Simulationszeit");
        } else {
            RCLCPP_WARN(this->get_logger(), "Nutze Wallclock-Zeit");
        }

        this->declare_parameter<double>("linear_speed", 1.5);
        this->declare_parameter<double>("k_p", 2.0);
        this->declare_parameter<double>("wheelbase", 0.365);
        this->declare_parameter<double>("max_steering_angle", 0.436332); // Max. Lenkwinkel (ca. 25 Grad) in Radiant
        this->declare_parameter<double>("min_abs_steering_for_effect", 0.0174533); // Min. effektiver Lenkwinkel (ca. 1 Grad) in Radiant
        this->declare_parameter<double>("heading_tolerance", 0.00872665); // Toleranz für Ziel-Heading (ca. 0.5 Grad) in Radiant

        angle_subscriber_ = this->create_subscription<std_msgs::msg::Float64>(
            "target/angle", 10,
            std::bind(&AngleNode::angle_callback, this, std::placeholders::_1));

        twist_publisher_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
            "/bicycle_steering_controller/reference", 10);

        last_time_ = this->now();

        control_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&AngleNode::control_loop, this));
    }

private:
    void angle_callback(const std_msgs::msg::Float64::SharedPtr msg) {
        remaining_heading_change_ += msg->data;
        if (std::abs(remaining_heading_change_) > this->get_parameter("heading_tolerance").as_double()) {
            drive_mode_ = DriveMode::TURNING;
        }
        RCLCPP_INFO(this->get_logger(), "Neuer Winkel: %.3f rad, Gesamt: %.3f rad",
            msg->data, remaining_heading_change_);
    }

    void control_loop() {
        auto current_time = this->now();
        double dt = (current_time - last_time_).seconds();
        last_time_ = current_time;

        if (dt <= 0.0) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                "Ungültiger dt: %.4f, überspringe Zyklus", dt);
            return;
        }

        double linear_speed = this->get_parameter("linear_speed").as_double();
        double k_p = this->get_parameter("k_p").as_double();
        double wheelbase = this->get_parameter("wheelbase").as_double();
        double max_steering_angle = this->get_parameter("max_steering_angle").as_double();
        double min_abs_steering_for_effect = this->get_parameter("min_abs_steering_for_effect").as_double();
        double heading_tolerance = this->get_parameter("heading_tolerance").as_double();

        double commanded_steering_angle = 0.0;

        if (std::abs(remaining_heading_change_) > heading_tolerance) {
            drive_mode_ = DriveMode::TURNING;
            commanded_steering_angle = k_p * remaining_heading_change_;

            if (commanded_steering_angle > max_steering_angle) {
                commanded_steering_angle = max_steering_angle;
            } else if (commanded_steering_angle < -max_steering_angle) {
                commanded_steering_angle = -max_steering_angle;
            }

            // Stellt sicher, dass der Lenkwinkel groß genug ist
            if (std::abs(commanded_steering_angle) > 1e-9 &&
                std::abs(commanded_steering_angle) < min_abs_steering_for_effect &&
                std::abs(remaining_heading_change_) > heading_tolerance) {
                commanded_steering_angle = sgn(commanded_steering_angle) * min_abs_steering_for_effect;

                if (commanded_steering_angle > max_steering_angle) commanded_steering_angle = max_steering_angle;
                if (commanded_steering_angle < -max_steering_angle) commanded_steering_angle = -max_steering_angle;
            }
        } else {
            commanded_steering_angle = 0.0;
            remaining_heading_change_ = 0.0;
            drive_mode_ = DriveMode::STRAIGHT;
        }

        double current_linear_speed = linear_speed;

        auto twist_msg = geometry_msgs::msg::TwistStamped();
        twist_msg.header.stamp = current_time;
        twist_msg.header.frame_id = "base_link";
        twist_msg.twist.linear.x = current_linear_speed;
        twist_msg.twist.angular.z = commanded_steering_angle;
        twist_publisher_->publish(twist_msg);

        double estimated_yaw_rate = 0.0;
        if (wheelbase > 1e-3 && std::abs(linear_speed) > 1e-3) {
            estimated_yaw_rate = (linear_speed / wheelbase) * std::tan(commanded_steering_angle);
        }

        double executed_heading_change_this_step = estimated_yaw_rate * dt;

        // Aktualisierung der verbleibenden Kursänderung
        if ( (remaining_heading_change_ > 0 && executed_heading_change_this_step > 0) ||
             (remaining_heading_change_ < 0 && executed_heading_change_this_step < 0) ) {
            if (std::abs(executed_heading_change_this_step) >= std::abs(remaining_heading_change_)) {
                remaining_heading_change_ = 0.0; 
            } else {
                remaining_heading_change_ -= executed_heading_change_this_step;
            }
        } else if (std::abs(remaining_heading_change_) <= heading_tolerance) {
            remaining_heading_change_ = 0.0;
        }
    }

    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr angle_subscriber_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_publisher_;
    rclcpp::TimerBase::SharedPtr control_timer_;
    rclcpp::Time last_time_;
    double remaining_heading_change_;
    DriveMode drive_mode_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<AngleNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

