#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"

#include <string>
#include <vector>
#include <map>

class Drive : public rclcpp::Node
{
public:
  Drive()
  : Node("drive")
  {
    // Parameter Deklaration
    // Struktur: <Achse>_<Komponente>_<Transformationstyp>
    // Beispiel: linear_x_coeff, linear_x_offset

    // Parameter für Koeffizienten und Offsets
    declare_parameters_();

    // Parameter für den Frame ID der ausgehenden Nachricht
    this->declare_parameter<std::string>("output_frame_id", "base_link");
    output_frame_id_ = this->get_parameter("output_frame_id").as_string();

    // Publisher für die transformierte TwistStamped Nachricht
    publisher_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
      "/ackermann_steering_controller/reference", 10);

    // Subscriber für die eingehende TwistStamped Nachricht
    subscription_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
      "platzhalter/reference", 10,
      std::bind(&Drive::topic_callback, this, std::placeholders::_1));

    // Callback für Parameter-Änderungen
    param_callback_handle_ = this->add_on_set_parameters_callback(
        std::bind(&Drive::parameters_callback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Drive Node gestartet. Lauscht auf 'platzhalter/reference' und publiziert auf '/ackermann_steering_controller/reference'.");
    RCLCPP_INFO(this->get_logger(), "Output frame_id: %s", output_frame_id_.c_str());
  }

private:
  struct TransformValue {
    double coeff;
    double offset;
  };

  // Deklariert alle notwendigen Parameter für Koeffizienten und Offsets
  void declare_parameters_()
  {
    const std::vector<std::string> components = {"linear_x", "linear_y", "linear_z", "angular_x", "angular_y", "angular_z"};
    for (const auto& comp : components) {
      this->declare_parameter<double>(comp + "_coeff", 1.0);
      this->declare_parameter<double>(comp + "_offset", 0.0);
      RCLCPP_INFO(this->get_logger(), "Parameter deklariert: %s_coeff (default: 1.0), %s_offset (default: 0.0)", comp.c_str(), comp.c_str());
    }
  }

  // Liest den aktuellen Wert eines Parameters
  TransformValue get_transform_params(const std::string& base_name)
  {
    TransformValue params;
    params.coeff = this->get_parameter(base_name + "_coeff").as_double();
    params.offset = this->get_parameter(base_name + "_offset").as_double();
    return params;
  }

  // Callback für eingehende TwistStamped Nachrichten
  void topic_callback(const geometry_msgs::msg::TwistStamped::SharedPtr msg)
  {
    RCLCPP_DEBUG(this->get_logger(), "Eingehende TwistStamped Nachricht empfangen.");

    geometry_msgs::msg::TwistStamped out_msg;
    out_msg.header.stamp = this->get_clock()->now();
    out_msg.header.frame_id = output_frame_id_;

    // Transformationen anwenden
    TransformValue params_lx = get_transform_params("linear_x");
    out_msg.twist.linear.x = msg->twist.linear.x * params_lx.coeff + params_lx.offset;

    TransformValue params_ly = get_transform_params("linear_y");
    out_msg.twist.linear.y = msg->twist.linear.y * params_ly.coeff + params_ly.offset;

    TransformValue params_lz = get_transform_params("linear_z");
    out_msg.twist.linear.z = msg->twist.linear.z * params_lz.coeff + params_lz.offset;

    TransformValue params_ax = get_transform_params("angular_x");
    out_msg.twist.angular.x = msg->twist.angular.x * params_ax.coeff + params_ax.offset;

    TransformValue params_ay = get_transform_params("angular_y");
    out_msg.twist.angular.y = msg->twist.angular.y * params_ay.coeff + params_ay.offset;

    TransformValue params_az = get_transform_params("angular_z");
    out_msg.twist.angular.z = msg->twist.angular.z * params_az.coeff + params_az.offset;

    publisher_->publish(out_msg);
    RCLCPP_DEBUG(this->get_logger(), "Transformierte TwistStamped Nachricht publiziert.");
  }

  // Callback für Parameter-Änderungen
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
      // Die Koeffizienten und Offsets werden bei Bedarf in topic_callback() neu geladen,
      // daher ist hier keine explizite Aktualisierung von Member-Variablen notwendig,
      // es sei denn, man möchte Validierungen durchführen.
    }
    return result;
  }

  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr subscription_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr publisher_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;
  std::string output_frame_id_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Drive>());
  rclcpp::shutdown();
  return 0;
}