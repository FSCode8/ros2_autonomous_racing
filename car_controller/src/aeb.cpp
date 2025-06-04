#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/int32.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp" // Hinzugefügt für Parameter-Callback

#include <string>   // Hinzugefügt für std::string
#include <vector>   // Hinzugefügt für std::vector
#include <chrono>   // Hinzugefügt für std::chrono_literals

using namespace std::chrono_literals;

// Klassendeklaration direkt in der .cpp Datei
class AEB_Node : public rclcpp::Node
{
public:
  AEB_Node(); // Konstruktor-Deklaration

private:
  // Methoden-Deklarationen
  void calculate_ttc();
  void distance_callback(const std_msgs::msg::Float32::SharedPtr msg);
  void velocity_callback(const geometry_msgs::msg::TwistStamped::SharedPtr msg);
  void emergency_callback(const std_msgs::msg::Int32::SharedPtr msg); // NEUE CALLBACK FÜR EMERGENCY RESET
  rcl_interfaces::msg::SetParametersResult parameters_callback(
    const std::vector<rclcpp::Parameter> &parameters);

  // Member-Variablen
  double distance;
  double velocity;
  double ttc;
  bool emergency_active;
  
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr ttc_publisher;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr emergency_publisher;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr dist_subscription;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr vel_subscription;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr emergency_subscription; // NEUER SUBSCRIBER
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;
};

// Konstruktor-Definition
AEB_Node::AEB_Node()
: Node("aeb"), distance(-1.0), velocity(0.0), ttc(-1.0), emergency_active(false)
{
  // Parameter für AEB-Funktionalität (1=aktiviert, 0=deaktiviert)
  this->declare_parameter<int>("aeb_aktiv", 1);
  
  // Parameter für TTC-Schwellwert (in Sekunden)
  this->declare_parameter<double>("ttc_threshold", 1.5);

  // Parameter für den Controller-Typ (bicycle oder ackermann) für das Geschwindigkeits-Topic
  this->declare_parameter<std::string>("controller_type", "bicycle");
  
  int aeb_aktiv = this->get_parameter("aeb_aktiv").as_int();
  double ttc_threshold = this->get_parameter("ttc_threshold").as_double();
  std::string controller_type = this->get_parameter("controller_type").as_string();

  // Publisher für TTC-Wert und Notaus
  ttc_publisher = this->create_publisher<std_msgs::msg::Float32>("/time_to_collision", 10);
  emergency_publisher = this->create_publisher<std_msgs::msg::Int32>("/emergency/braking", 10);

  // Subscriber für Distanzmessung
  dist_subscription = this->create_subscription<std_msgs::msg::Float32>(
    "/collision_distance", 10, 
    std::bind(&AEB_Node::distance_callback, this, std::placeholders::_1));
    
  // Subscriber für Emergency-Braking-Topic, um externe Resets zu erkennen
  emergency_subscription = this->create_subscription<std_msgs::msg::Int32>(
    "/emergency/braking", 10, 
    std::bind(&AEB_Node::emergency_callback, this, std::placeholders::_1));
  
  // Geschwindigkeits-Topic basierend auf controller_type bestimmen
  std::string velocity_topic;
  if (controller_type == "bicycle") {
    velocity_topic = "/bicycle_steering_controller/reference";
  } else if (controller_type == "ackermann") {
    velocity_topic = "/ackermann_steering_controller/reference";
  } else {
    RCLCPP_WARN(this->get_logger(), 
               "Unbekannter controller_type '%s' für Geschwindigkeits-Topic, verwende 'bicycle' als Fallback", 
               controller_type.c_str());
    velocity_topic = "/bicycle_steering_controller/reference";
    // Optional: controller_type auf den Fallback-Wert setzen, wenn es für andere Logik relevant ist
    // controller_type = "bicycle"; 
  }
    
  // Subscriber für Geschwindigkeit
  vel_subscription = this->create_subscription<geometry_msgs::msg::TwistStamped>(
    velocity_topic, 10, 
    std::bind(&AEB_Node::velocity_callback, this, std::placeholders::_1));

  // Parameter-Callback für Laufzeit-Änderungen
  param_callback_handle_ = this->add_on_set_parameters_callback(
    std::bind(&AEB_Node::parameters_callback, this, std::placeholders::_1));

  RCLCPP_INFO(this->get_logger(), "AEB Node gestartet.");
  RCLCPP_INFO(this->get_logger(), "AEB Status: %s (Parameter 'aeb_aktiv'=%d)", 
               aeb_aktiv == 1 ? "AKTIVIERT" : "DEAKTIVIERT", aeb_aktiv);
  RCLCPP_INFO(this->get_logger(), "TTC-Schwellwert: %.2f Sekunden", ttc_threshold);
  RCLCPP_INFO(this->get_logger(), "Lauscht auf Kollisionsdistanz auf '/collision_distance'.");
  RCLCPP_INFO(this->get_logger(), "Lauscht auf Geschwindigkeit auf '%s' (Controller-Typ: %s).", 
              velocity_topic.c_str(), controller_type.c_str());
  RCLCPP_INFO(this->get_logger(), "Standardmäßig ist AEB aktiviert. Deaktivierung über: ros2 param set /aeb aeb_aktiv 0");
  RCLCPP_INFO(this->get_logger(), "Nach einer Notbremsung muss manuell zurückgesetzt werden: ros2 topic pub /emergency/braking std_msgs/msg/Int32 \"{data: 0}\" --once");
}

// Methoden-Definitionen
void AEB_Node::calculate_ttc()
{
  std_msgs::msg::Float32 ttc_msg;
  bool aeb_enabled = this->get_parameter("aeb_aktiv").as_int() == 1;
  double ttc_threshold = this->get_parameter("ttc_threshold").as_double();
  
  RCLCPP_INFO(this->get_logger(), "calculate_ttc: distance=%.2f, velocity=%.2f, aeb_enabled=%d, ttc_threshold=%.2f, emergency_active=%d",
              distance, velocity, aeb_enabled, ttc_threshold, static_cast<int>(emergency_active));


  // TTC berechnen
  if(distance <= 0.0) { // Wenn Distanz ungültig oder 0
    ttc = -1.0; // TTC als ungültig markieren
    // Optional: ttc_msg hier mit -1.0 publishen, um den ungültigen Status anzuzeigen
    // ttc_msg.data = ttc;
    // ttc_publisher->publish(ttc_msg);
  }
  else if(velocity > 0.0) { // Nur wenn sich das Fahrzeug vorwärts bewegt
    ttc = distance / velocity;
    
    // TTC immer publishen
    ttc_msg.data = ttc;
    ttc_publisher->publish(ttc_msg);
    RCLCPP_INFO(this->get_logger(), "Published TTC: %.2f", ttc);
    
    // Notbremsung auslösen wenn:
    // 1. AEB aktiviert ist
    // 2. TTC unter Schwellwert fällt
    // 3. Keine Notbremsung bereits aktiv ist
    if(aeb_enabled && ttc < ttc_threshold && !emergency_active) {
      RCLCPP_WARN(this->get_logger(), 
        "NOTBREMSUNG AUSGELÖST! TTC (%.2f s) unter Schwellwert (%.2f s)", ttc, ttc_threshold);
      
      std_msgs::msg::Int32 brake_msg;
      brake_msg.data = 1;
      emergency_publisher->publish(brake_msg);
      emergency_active = true; // Notbremsung als aktiv markieren
    }
  } else { // Wenn velocity <= 0 (steht oder fährt rückwärts)
      ttc = -2.0; // Anderer ungültiger Wert, um diesen Fall zu kennzeichnen, oder einfach -1.0
      RCLCPP_INFO(this->get_logger(), "Velocity is <= 0 (%.2f), TTC not calculated for braking.", velocity);
      // Optional: ttc_msg hier mit dem entsprechenden Wert publishen
      // ttc_msg.data = ttc;
      // ttc_publisher->publish(ttc_msg);
  }
}

void AEB_Node::distance_callback(const std_msgs::msg::Float32::SharedPtr msg)
{
  RCLCPP_INFO(this->get_logger(), "Distance callback triggered. Received distance: %.2f", msg->data);
  distance = msg->data;
  if (this->get_parameter("aeb_aktiv").as_int() == 0) {
      emergency_active = false;
  }
  RCLCPP_INFO(this->get_logger(), "Calling calculate_ttc from distance_callback. Current velocity: %.2f", velocity);
  calculate_ttc();
}

void AEB_Node::velocity_callback(const geometry_msgs::msg::TwistStamped::SharedPtr msg)
{
  RCLCPP_INFO(this->get_logger(), "Velocity callback triggered. Received linear.x: %.2f from topic: %s", 
              msg->twist.linear.x, vel_subscription->get_topic_name());
  velocity = msg->twist.linear.x;
  if (this->get_parameter("aeb_aktiv").as_int() == 0) {
      emergency_active = false;
  }
  RCLCPP_INFO(this->get_logger(), "Calling calculate_ttc from velocity_callback. Current distance: %.2f", distance);
  calculate_ttc();
}

void AEB_Node::emergency_callback(const std_msgs::msg::Int32::SharedPtr msg)
{
  RCLCPP_INFO(this->get_logger(), "Emergency callback triggered. Received value: %d", msg->data);
  
  // Falls Wert 0 empfangen wird, Notbremsstatus zurücksetzen
  if (msg->data == 0 && emergency_active) {
    emergency_active = false;
    RCLCPP_INFO(this->get_logger(), "Notbremsstatus zurückgesetzt durch externe Nachricht auf /emergency/braking");
  }
  // Falls Wert 1 empfangen wird und nicht von uns selbst kommt, Notbremsstatus aktivieren
  else if (msg->data == 1 && !emergency_active) {
    emergency_active = true;
    RCLCPP_WARN(this->get_logger(), "Notbremsung durch externe Nachricht auf /emergency/braking aktiviert");
  }
}

rcl_interfaces::msg::SetParametersResult AEB_Node::parameters_callback(
  const std::vector<rclcpp::Parameter> &parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "success";

  for (const auto &param : parameters) {
    if (param.get_name() == "aeb_aktiv") {
      RCLCPP_INFO(this->get_logger(), "AEB %s", 
                  param.as_int() == 1 ? "aktiviert" : "deaktiviert");
      if (param.as_int() == 0) {
        emergency_active = false; // Wenn AEB deaktiviert wird, auch den Notbremsstatus zurücksetzen
        RCLCPP_INFO(this->get_logger(), "Notbremsstatus zurückgesetzt, da AEB deaktiviert wurde.");
      }
    }
    else if (param.get_name() == "ttc_threshold") {
      RCLCPP_INFO(this->get_logger(), "TTC-Schwellwert auf %.2f Sekunden gesetzt", 
                  param.as_double());
    }
  }
  return result;
}

// main Funktion
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AEB_Node>());
  rclcpp::shutdown();
  return 0;
}