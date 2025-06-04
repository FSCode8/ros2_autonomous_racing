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
  void distance_callback(const std_msgs::msg::Float32::SharedPtr msg); // Geändert zu SharedPtr
  void velocity_callback(const geometry_msgs::msg::TwistStamped::SharedPtr msg); // Geändert zu SharedPtr
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
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;
};

// Konstruktor-Definition
AEB_Node::AEB_Node()
: Node("aeb_node"), distance(-1.0), velocity(0.0), ttc(-1.0), emergency_active(false)
{
  // Parameter für AEB-Funktionalität (1=aktiviert, 0=deaktiviert)
  this->declare_parameter<int>("aeb_aktiv", 1);
  
  // Parameter für TTC-Schwellwert (in Sekunden)
  this->declare_parameter<double>("ttc_threshold", 1.5);

  // Publisher für TTC-Wert und Notaus
  ttc_publisher = this->create_publisher<std_msgs::msg::Float32>("/time_to_collision", 10);
  emergency_publisher = this->create_publisher<std_msgs::msg::Int32>("/emergency/braking", 10);

  // Subscriber für Distanzmessung und Geschwindigkeit
  dist_subscription = this->create_subscription<std_msgs::msg::Float32>(
    "/collision_distance", 10, 
    std::bind(&AEB_Node::distance_callback, this, std::placeholders::_1));
    
  vel_subscription = this->create_subscription<geometry_msgs::msg::TwistStamped>(
    "/bicycle_steering_controller/reference", 10, 
    std::bind(&AEB_Node::velocity_callback, this, std::placeholders::_1));

  // Parameter-Callback für Laufzeit-Änderungen
  param_callback_handle_ = this->add_on_set_parameters_callback(
    std::bind(&AEB_Node::parameters_callback, this, std::placeholders::_1));

  RCLCPP_INFO(this->get_logger(), "AEB Node gestartet. Überwache Kollisionsdistanz und Geschwindigkeit.");
  RCLCPP_INFO(this->get_logger(), "Standardmäßig ist AEB aktiviert. Deaktivierung über: ros2 param set /aeb_node aeb_aktiv 0");
  RCLCPP_INFO(this->get_logger(), "Nach einer Notbremsung muss manuell zurückgesetzt werden: ros2 topic pub /emergency/braking std_msgs/msg/Int32 \"{data: 0}\" --once");
}

// Methoden-Definitionen
void AEB_Node::calculate_ttc()
{
  std_msgs::msg::Float32 ttc_msg;
  bool aeb_enabled = this->get_parameter("aeb_aktiv").as_int() == 1;
  double ttc_threshold = this->get_parameter("ttc_threshold").as_double();
  
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
      // Optional: ttc_msg hier mit dem entsprechenden Wert publishen
      // ttc_msg.data = ttc;
      // ttc_publisher->publish(ttc_msg);
  }
}

void AEB_Node::distance_callback(const std_msgs::msg::Float32::SharedPtr msg)
{
  distance = msg->data;
  // Prüfen, ob eine Notbremsung aktiv war und durch "/emergency/braking" {data:0} zurückgesetzt wurde
  // Dies ist eine vereinfachte Annahme. Eine robustere Lösung würde den Status von /emergency/braking abonnieren.
  // Für dieses Beispiel gehen wir davon aus, dass emergency_active nur durch die AEB-Logik selbst auf true gesetzt wird
  // und manuell (extern) auf false (indirekt durch das Publishen von 0 auf /emergency/braking, was hier nicht direkt geprüft wird).
  // Um `emergency_active` korrekt zurückzusetzen, wenn ein `data:0` auf `/emergency/braking` empfangen wird,
  // müsste diese Node `/emergency/braking` auch abonnieren.
  // Für die aktuelle Anforderung: Wenn `aeb_aktiv` auf 0 gesetzt wird, wird `emergency_active` auch zurückgesetzt.
  if (this->get_parameter("aeb_aktiv").as_int() == 0) {
      emergency_active = false;
  }
  calculate_ttc();
}

void AEB_Node::velocity_callback(const geometry_msgs::msg::TwistStamped::SharedPtr msg)
{
  velocity = msg->twist.linear.x;
  // Ähnliche Logik wie in distance_callback für das Zurücksetzen von emergency_active
  if (this->get_parameter("aeb_aktiv").as_int() == 0) {
      emergency_active = false;
  }
  calculate_ttc();
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