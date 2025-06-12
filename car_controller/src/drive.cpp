#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "std_msgs/msg/int32.hpp"

#include <string>
#include <vector>
#include <map>
#include <limits>

class Drive : public rclcpp::Node
{
public:
  Drive()
  : Node("drive"), emergency_stop_active_(false)
  {
    // Parameter für den Controller-Typ deklarieren und abrufen
    this->declare_parameter<std::string>("controller_type", "bicycle");
    std::string controller_type = this->get_parameter("controller_type").as_string();

    // Transfomationsparameter für alle Komponenten deklarieren
    declare_parameters_();

    // Parameter für die Output-Frame-ID deklarieren und abrufen
    this->declare_parameter<std::string>("output_frame_id", "base_link");
    output_frame_id_ = this->get_parameter("output_frame_id").as_string();

    // Publisher-Topic basierend auf dem Controller-Typ bestimmen
    std::string topic;
    if (controller_type == "bicycle") {
      topic = "/bicycle_steering_controller/reference";
    } else if (controller_type == "ackermann") {
      topic = "/ackermann_steering_controller/reference";
    } else {
      RCLCPP_WARN(this->get_logger(), 
                 "Unbekannter controller_type '%s', verwende 'bicycle' als Standard", 
                 controller_type.c_str());
      topic = "/bicycle_steering_controller/reference";
      controller_type = "bicycle";
    }

    // Publisher für transformierte TwistStamped-Nachrichten erstellen
    publisher_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(topic, 10);

    // Subscriber für eingehende TwistStamped-Nachrichten erstellen
    subscription_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
      "raw_drive_commands", 10,
      std::bind(&Drive::topic_callback, this, std::placeholders::_1));

    // Subscriber für Not-Aus-Nachrichten erstellen
    emergency_stop_subscriber_ = this->create_subscription<std_msgs::msg::Int32>(
      "/emergency/braking", 10,
      std::bind(&Drive::emergency_stop_callback, this, std::placeholders::_1));

    // Callback für Parameteränderungen registrieren
    param_callback_handle_ = this->add_on_set_parameters_callback(
        std::bind(&Drive::parameters_callback, this, std::placeholders::_1));

    // Debugausgabe
    RCLCPP_INFO(this->get_logger(), "Drive Node gestartet. Lauscht auf 'raw_drive_commands' und publiziert auf '%s'.", 
                topic.c_str());
    RCLCPP_INFO(this->get_logger(), "Controller-Typ: %s", controller_type.c_str());
    RCLCPP_INFO(this->get_logger(), "Lauscht auf Not-Aus-Signale auf '/emergency/braking'.");
    RCLCPP_INFO(this->get_logger(), "Output frame_id: %s", output_frame_id_.c_str());
  }

private:
  struct TransformValue {
    double coeff;
    double offset;
    double min_val;
    double max_val;
  };

  // Parameterdeklaration
  void declare_parameters_()
  {
    const std::vector<std::string> components = {"linear_x", "linear_y", "linear_z", "angular_x", "angular_y", "angular_z"};
    for (const auto& comp : components) {
      this->declare_parameter<double>(comp + "_coeff", 1.0);
      this->declare_parameter<double>(comp + "_offset", 0.0);
      this->declare_parameter<double>(comp + "_min", -std::numeric_limits<double>::infinity());
      this->declare_parameter<double>(comp + "_max", std::numeric_limits<double>::infinity());
      RCLCPP_INFO(this->get_logger(), "Parameter deklariert: %s_coeff, %s_offset, %s_min, %s_max", comp.c_str(), comp.c_str(), comp.c_str(), comp.c_str());
    }
  }

  // Liest die Transformationsparameter für eine bestimmte Komponente
  TransformValue get_transform_params(const std::string& base_name)
  {
    TransformValue params;
    params.coeff = this->get_parameter(base_name + "_coeff").as_double();
    params.offset = this->get_parameter(base_name + "_offset").as_double();
    params.min_val = this->get_parameter(base_name + "_min").as_double();
    params.max_val = this->get_parameter(base_name + "_max").as_double();
    return params;
  }

  // Begrenzt einen Wert zwischen einem Minimum und Maximum
  double clamp(double value, double min_val, double max_val) {
    return std::max(min_val, std::min(value, max_val));
  }

  // Manueller / AEB- Not-Aus
  void emergency_stop_callback(const std_msgs::msg::Int32::SharedPtr msg)
  {
    if (msg->data == 1 && !emergency_stop_active_) {
      emergency_stop_active_ = true;
      RCLCPP_WARN(this->get_logger(), "NOT-AUS AKTIVIERT! Alle Befehle werden ignoriert und Fahrzeug gestoppt.");
      geometry_msgs::msg::TwistStamped stop_msg;
      stop_msg.header.stamp = this->get_clock()->now();
      stop_msg.header.frame_id = output_frame_id_;
      stop_msg.twist.linear.x = 0.0;
      stop_msg.twist.linear.y = 0.0;
      stop_msg.twist.linear.z = 0.0;
      stop_msg.twist.angular.x = 0.0;
      stop_msg.twist.angular.y = 0.0;
      stop_msg.twist.angular.z = 0.0;
      publisher_->publish(stop_msg);
    } else if (msg->data == 0 && emergency_stop_active_) {
      emergency_stop_active_ = false;
      RCLCPP_INFO(this->get_logger(), "Not-Aus deaktiviert. Befehlsverarbeitung fortgesetzt.");
    }
  }

  void topic_callback(const geometry_msgs::msg::TwistStamped::SharedPtr msg)
  {
    if (emergency_stop_active_) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "Not-Aus ist aktiv. Eingehende Befehle werden ignoriert.");
      return;
    }

    RCLCPP_DEBUG(this->get_logger(), "Eingehende TwistStamped Nachricht empfangen.");

    geometry_msgs::msg::TwistStamped out_msg;
    out_msg.header.stamp = this->get_clock()->now();
    out_msg.header.frame_id = output_frame_id_;

    // Transformationen auf die einzelnen Komponenten anwenden
    TransformValue params_lx = get_transform_params("linear_x");
    double transformed_lx = msg->twist.linear.x * params_lx.coeff + params_lx.offset;
    out_msg.twist.linear.x = clamp(transformed_lx, params_lx.min_val, params_lx.max_val);

    TransformValue params_ly = get_transform_params("linear_y");
    double transformed_ly = msg->twist.linear.y * params_ly.coeff + params_ly.offset;
    out_msg.twist.linear.y = clamp(transformed_ly, params_ly.min_val, params_ly.max_val);

    TransformValue params_lz = get_transform_params("linear_z");
    double transformed_lz = msg->twist.linear.z * params_lz.coeff + params_lz.offset;
    out_msg.twist.linear.z = clamp(transformed_lz, params_lz.min_val, params_lz.max_val);

    TransformValue params_ax = get_transform_params("angular_x");
    double transformed_ax = msg->twist.angular.x * params_ax.coeff + params_ax.offset;
    out_msg.twist.angular.x = clamp(transformed_ax, params_ax.min_val, params_ax.max_val);

    TransformValue params_ay = get_transform_params("angular_y");
    double transformed_ay = msg->twist.angular.y * params_ay.coeff + params_ay.offset;
    out_msg.twist.angular.y = clamp(transformed_ay, params_ay.min_val, params_ay.max_val);

    TransformValue params_az = get_transform_params("angular_z");
    double transformed_az = msg->twist.angular.z * params_az.coeff + params_az.offset;
    out_msg.twist.angular.z = clamp(transformed_az, params_az.min_val, params_az.max_val);

    publisher_->publish(out_msg);
    RCLCPP_DEBUG(this->get_logger(), "Transformierte TwistStamped Nachricht publiziert.");
  }

  // Verarbeitet Änderungen an Parametern
  rcl_interfaces::msg::SetParametersResult parameters_callback(
    const std::vector<rclcpp::Parameter> &parameters)
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    result.reason = "success";

    for (const auto &param : parameters) {
      RCLCPP_INFO(this->get_logger(), "Parameter '%s' geändert zu '%s'",
                  param.get_name().c_str(), param.value_to_string().c_str());
      if (param.get_name() == "output_frame_id") {
        output_frame_id_ = param.as_string();
      }
    }
    return result;
  }

  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr subscription_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr publisher_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr emergency_stop_subscriber_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;
  std::string output_frame_id_;
  bool emergency_stop_active_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Drive>());
  rclcpp::shutdown();
  return 0;
}