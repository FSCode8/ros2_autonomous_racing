#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "std_msgs/msg/float64.hpp"
#include <cmath> // Für std::abs und std::tan

// Hilfsfunktion für das Vorzeichen einer Zahl
template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}

class BicycleControlNode : public rclcpp::Node {
public:
  BicycleControlNode() : Node("bicycle_control_node"), remaining_heading_change_(0.0) {
    // WICHTIG: Parameter für die Verwendung der Simulationszeit deklarieren
    this->declare_parameter<bool>("use_sim_time", false); // Standardmäßig false, sollte aber für Gazebo auf true gesetzt werden
    bool use_sim_time = this->get_parameter("use_sim_time").as_bool();

    if (use_sim_time) {
      RCLCPP_INFO(this->get_logger(), "Parameter 'use_sim_time' ist true. Simulationszeit wird verwendet.");
    } else {
      RCLCPP_WARN(this->get_logger(), "Parameter 'use_sim_time' ist false oder nicht gesetzt. Wall-Clock-Zeit wird verwendet. Dies kann zu Problemen mit Gazebo führen! Starten Sie den Knoten mit 'use_sim_time:=true'.");
    }

    // Parameter deklarieren und initialisieren
    this->declare_parameter<double>("linear_speed", 1.5); // Lineare Geschwindigkeit in m/s
    this->declare_parameter<double>("k_p", 2.0);          // Proportionalverstärkung für den Lenkregler. MUSS ggf. ERHÖHT werden.
    this->declare_parameter<double>("wheelbase", 1.0);    // Radstand des Fahrzeugs in Metern. SEHR WICHTIG für korrekte Kinematik.
    this->declare_parameter<double>("max_steering_angle", 0.436332); // Max. Lenkwinkel (ca. 25 Grad) in Radiant
    this->declare_parameter<double>("min_abs_steering_for_effect", 0.0174533); // Min. effektiver Lenkwinkel (ca. 1 Grad) in Radiant
    this->declare_parameter<double>("heading_tolerance", 0.00872665); // Toleranz für Ziel-Heading (ca. 0.5 Grad) in Radiant

    angle_subscriber_ = this->create_subscription<std_msgs::msg::Float64>(
      "target/angle", 10,
      std::bind(&BicycleControlNode::angle_callback, this, std::placeholders::_1));

    twist_publisher_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
      "/ackermann_steering_controller/reference", 10); // Stellen Sie sicher, dass dies das korrekte Topic für Ihren Gazebo Ackermann Controller ist

    // Initialisiere last_time_ für eine dynamische dt-Berechnung
    // this->now() gibt entweder Wall-Clock oder Sim-Zeit zurück, abhängig von use_sim_time
    last_time_ = this->now();

    // Control loop timer
    control_timer_ = this->create_wall_timer( // Der Timer selbst läuft immer noch auf Wall-Clock-Basis
      std::chrono::milliseconds(100),         // aber die dt-Berechnung im Loop verwendet this->now()
      std::bind(&BicycleControlNode::control_loop, this));

    RCLCPP_INFO(this->get_logger(), "BicycleControlNode gestartet. Überprüfen Sie 'use_sim_time' und Fahrzeugparameter.");
  }

private:
  void angle_callback(const std_msgs::msg::Float64::SharedPtr msg) {
    remaining_heading_change_ += msg->data; // Eingehende relative Winkelanforderung aufaddieren
    RCLCPP_INFO(this->get_logger(),
      "Neue relative Kursanforderung erhalten: %.3f rad. Verbleibende Gesamtänderung: %.3f rad",
      msg->data, remaining_heading_change_);
  }

  void control_loop() {
    auto current_time = this->now(); // Verwendet Sim-Zeit, wenn use_sim_time=true
    double dt = (current_time - last_time_).seconds();
    last_time_ = current_time;

    if (dt <= 0.0) {
      // Dies kann passieren, wenn die Simulationszeit beim Start noch nicht richtig initialisiert ist
      // oder wenn die Simulation pausiert ist und dann fortgesetzt wird mit demselben Zeitstempel.
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, // Loggt max. einmal pro Sekunde
        "Ungültiger oder Null dt-Wert: %.4f. Kontrollschleife wird übersprungen. Ist die Simulation aktiv und use_sim_time korrekt gesetzt?", dt);
      return;
    }

    // Parameter dynamisch auslesen
    double linear_speed = this->get_parameter("linear_speed").as_double();
    double k_p = this->get_parameter("k_p").as_double();
    double wheelbase = this->get_parameter("wheelbase").as_double();
    double max_steering_angle = this->get_parameter("max_steering_angle").as_double();
    double min_abs_steering_for_effect = this->get_parameter("min_abs_steering_for_effect").as_double();
    double heading_tolerance = this->get_parameter("heading_tolerance").as_double();

    double commanded_steering_angle = 0.0;

    if (std::abs(remaining_heading_change_) > heading_tolerance) {
      commanded_steering_angle = k_p * remaining_heading_change_;

      if (commanded_steering_angle > max_steering_angle) {
        commanded_steering_angle = max_steering_angle;
      } else if (commanded_steering_angle < -max_steering_angle) {
        commanded_steering_angle = -max_steering_angle;
      }

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
    }

    auto twist_msg = geometry_msgs::msg::TwistStamped();
    twist_msg.header.stamp = current_time; 
    twist_msg.header.frame_id = "base_link"; 
    twist_msg.twist.linear.x = linear_speed;
    twist_msg.twist.angular.z = commanded_steering_angle; 
    twist_publisher_->publish(twist_msg);

    double estimated_yaw_rate = 0.0;
    if (wheelbase > 1e-3 && std::abs(linear_speed) > 1e-3) { // Vermeide Division durch Null oder unrealistische Werte
      estimated_yaw_rate = (linear_speed / wheelbase) * std::tan(commanded_steering_angle);
    } else if (std::abs(remaining_heading_change_) > heading_tolerance) {
        // Wenn wir uns drehen sollen, aber voraussichtlich keine Gierrate haben (z.B. v=0), loggen.
        RCLCPP_DEBUG(this->get_logger(), "Gierrate kann nicht geschätzt werden: wheelbase=%.2f, linear_speed=%.2f, commanded_steering_angle=%.2f",
            wheelbase, linear_speed, commanded_steering_angle);
    }


    double executed_heading_change_this_step = estimated_yaw_rate * dt;

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
    // Wenn executed_heading_change_this_step = 0 (z.B. v=0, oder Lenkwinkel=0)
    // oder das falsche Vorzeichen hat (sollte bei diesem P-Regler nicht passieren, es sei denn, k_p ist negativ),
    // wird remaining_heading_change_ nicht wie erwartet reduziert.

    RCLCPP_DEBUG(this->get_logger(),
      "SimTime: %.3f, dt: %.4fs, LinSpd: %.2fm/s, Wheelbase: %.2fm, k_p: %.2f",
      current_time.seconds(), dt, linear_speed, wheelbase, k_p);
    // Korrigierte Debug-Zeile: this->get_parameter("target/angle")... entfernt
    RCLCPP_DEBUG(this->get_logger(),
      "RemHeadChg: %.3frad, RawSteerCmd(k_p*err): %.3frad, FinalCmdSteerAng: %.3frad",
      remaining_heading_change_, k_p * remaining_heading_change_, commanded_steering_angle);
    RCLCPP_DEBUG(this->get_logger(),
      "EstYawRate: %.3frad/s, ExecHeadChgStep: %.3frad, NewRemHeadChg: %.3frad",
      estimated_yaw_rate, executed_heading_change_this_step, remaining_heading_change_);
  }

  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr angle_subscriber_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_publisher_;
  rclcpp::TimerBase::SharedPtr control_timer_;
  
  rclcpp::Time last_time_;              
  double remaining_heading_change_;     
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  // Wichtig: use_sim_time muss oft global oder zumindest für den Executor gesetzt werden,
  // bevor der Node instanziiert wird, wenn es über Parameter-Datei geladen wird.
  // Wenn per Kommandozeile (--ros-args -p use_sim_time:=true), wird es korrekt behandelt.
  auto node = std::make_shared<BicycleControlNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
