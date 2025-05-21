#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "std_msgs/msg/float64.hpp"
#include <cmath> // Für std::abs und std::tan

// Hilfsfunktion für das Vorzeichen einer Zahl
template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}

class ModelCarBicycleControlNode : public rclcpp::Node {
public:
    // Neue Enum für den Fahrzustand
    enum class DriveMode {
        STOPPED,     // Fahrzeug steht
        TURNING,     // Fahrzeug führt Richtungsänderung durch
        STRAIGHT     // Fahrzeug fährt geradeaus
    };

    ModelCarBicycleControlNode() : Node("model_car_bicycle_control_node"), remaining_heading_change_(0.0), drive_mode_(DriveMode::STRAIGHT) {
        // Parameter deklarieren und initialisieren
        this->declare_parameter<double>("linear_speed", 1.5); // Lineare Geschwindigkeit in m/s
        this->declare_parameter<double>("k_p", 2.0);          // Proportionalverstärkung für den Lenkregler. MUSS ggf. ERHÖHT werden.
        this->declare_parameter<double>("wheelbase", 0.365);      // Radstand des Fahrzeugs in Metern. SEHR WICHTIG für korrekte Kinematik.
        this->declare_parameter<double>("max_steering_angle", 0.436332); // Max. Lenkwinkel (ca. 25 Grad) in Radiant
        this->declare_parameter<double>("min_abs_steering_for_effect", 0.0174533); // Min. effektiver Lenkwinkel (ca. 1 Grad) in Radiant
        this->declare_parameter<double>("heading_tolerance", 0.00872665); // Toleranz für Ziel-Heading (ca. 0.5 Grad) in Radiant

        angle_subscriber_ = this->create_subscription<std_msgs::msg::Float64>(
            "target/angle", 10,
            std::bind(&ModelCarBicycleControlNode::angle_callback, this, std::placeholders::_1));

        twist_publisher_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
            "/bicycle_steering_controller/reference", 10); // Geändertes Topic für den Controller

        // Initialisiere last_time_ für eine dynamische dt-Berechnung
        last_time_ = this->now();

        // Control loop timer
        control_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&ModelCarBicycleControlNode::control_loop, this));

        RCLCPP_INFO(this->get_logger(), "ModelCarBicycleControlNode gestartet.");
    }

private:
    void angle_callback(const std_msgs::msg::Float64::SharedPtr msg) {
        remaining_heading_change_ += msg->data; // Eingehende relative Winkelanforderung aufaddieren
        if (std::abs(remaining_heading_change_) > this->get_parameter("heading_tolerance").as_double()) {
            drive_mode_ = DriveMode::TURNING; // Wechsle in den Drehungsmodus, wenn nötig
        }
        RCLCPP_INFO(this->get_logger(),
            "Neue relative Kursanforderung erhalten: %.3f rad. Verbleibende Gesamtänderung: %.3f rad",
            msg->data, remaining_heading_change_);
    }

    void control_loop() {
        auto current_time = this->now();
        double dt = (current_time - last_time_).seconds();
        last_time_ = current_time;

        if (dt <= 0.0) {
            // Dies kann passieren, wenn die Simulationszeit beim Start noch nicht richtig initialisiert ist
            // oder wenn die Simulation pausiert ist und dann fortgesetzt wird mit demselben Zeitstempel.
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, // Loggt max. einmal pro Sekunde
                "Ungültiger oder Null dt-Wert: %.4f. Kontrollschleife wird übersprungen.", dt);
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
            drive_mode_ = DriveMode::TURNING;
            commanded_steering_angle = k_p * remaining_heading_change_;

            // Begrenzung des Lenkwinkels auf den Maximalwert
            if (commanded_steering_angle > max_steering_angle) {
                commanded_steering_angle = max_steering_angle;
            } else if (commanded_steering_angle < -max_steering_angle) {
                commanded_steering_angle = -max_steering_angle;
            }

            // Sicherstellen, dass ein minimaler Lenkwinkel angewendet wird, wenn eine Drehung erforderlich ist
            // und der berechnete Winkel sehr klein, aber nicht null ist.
            if (std::abs(commanded_steering_angle) > 1e-9 && // Nur wenn der Winkel nicht praktisch null ist
                std::abs(commanded_steering_angle) < min_abs_steering_for_effect &&
                std::abs(remaining_heading_change_) > heading_tolerance) { // Nur wenn wir uns tatsächlich drehen wollen
                commanded_steering_angle = sgn(commanded_steering_angle) * min_abs_steering_for_effect;
                
                // Erneute Begrenzung, falls min_abs_steering_for_effect > max_steering_angle (unwahrscheinlich, aber sicher ist sicher)
                if (commanded_steering_angle > max_steering_angle) commanded_steering_angle = max_steering_angle;
                if (commanded_steering_angle < -max_steering_angle) commanded_steering_angle = -max_steering_angle;
            }
        } else {
            // Ziel erreicht oder keine Änderung angefordert
            commanded_steering_angle = 0.0;
            remaining_heading_change_ = 0.0; // Setze verbleibende Änderung zurück
            
            // Setze den Modus explizit auf STRAIGHT, um geradeaus zu fahren
            drive_mode_ = DriveMode::STRAIGHT;
        }

        // Setze die Geschwindigkeit basierend auf dem Fahrmodus
        double current_linear_speed = linear_speed;
        
        // Erstelle und sende die Twist-Nachricht, unabhängig vom Modus
        auto twist_msg = geometry_msgs::msg::TwistStamped();
        twist_msg.header.stamp = current_time; // Zeitstempel setzen
        twist_msg.header.frame_id = "base_link"; // Oder der entsprechende Frame Ihres Roboters
        twist_msg.twist.linear.x = current_linear_speed; // Verwendet die Geschwindigkeit entsprechend dem Modus
        twist_msg.twist.angular.z = commanded_steering_angle; 
        twist_publisher_->publish(twist_msg);

        // Schätzung der Kursänderung in diesem Zeitschritt
        double estimated_yaw_rate = 0.0;
        if (wheelbase > 1e-3 && std::abs(linear_speed) > 1e-3) { // Vermeide Division durch Null oder unrealistische Werte
            // Kinematische Beziehung: Gierrate = (v / L) * tan(delta)
            estimated_yaw_rate = (linear_speed / wheelbase) * std::tan(commanded_steering_angle);
        } else if (std::abs(remaining_heading_change_) > heading_tolerance) {
             // Wenn wir uns drehen sollen, aber voraussichtlich keine Gierrate haben (z.B. v=0), loggen.
             RCLCPP_DEBUG(this->get_logger(), "Gierrate kann nicht geschätzt werden: wheelbase=%.2f, linear_speed=%.2f, commanded_steering_angle=%.2f",
                          wheelbase, linear_speed, commanded_steering_angle);
        }


        double executed_heading_change_this_step = estimated_yaw_rate * dt;

        // Reduziere remaining_heading_change_ basierend auf der geschätzten Ausführung
        // Nur reduzieren, wenn die geschätzte Änderung in die richtige Richtung geht
        if ( (remaining_heading_change_ > 0 && executed_heading_change_this_step > 0) ||
             (remaining_heading_change_ < 0 && executed_heading_change_this_step < 0) ) {
            if (std::abs(executed_heading_change_this_step) >= std::abs(remaining_heading_change_)) {
                // Wenn mehr oder gleich viel ausgeführt wurde als nötig, ist das Ziel erreicht
                remaining_heading_change_ = 0.0; 
            } else {
                remaining_heading_change_ -= executed_heading_change_this_step;
            }
        } else if (std::abs(remaining_heading_change_) <= heading_tolerance) {
            // Wenn wir schon sehr nah am Ziel sind, auf Null setzen.
            // Dies behandelt auch den Fall, dass executed_heading_change_this_step = 0 ist (z.B. v=0)
            // und wir uns innerhalb der Toleranz befinden.
            remaining_heading_change_ = 0.0;
        }

        // Log den aktuellen Fahrmodus
        if (drive_mode_ == DriveMode::TURNING) {
            RCLCPP_DEBUG(this->get_logger(), "Fahrmodus: TURNING, verbleibende Änderung: %.3f rad", 
                        remaining_heading_change_);
        } else if (drive_mode_ == DriveMode::STRAIGHT) {
            RCLCPP_DEBUG(this->get_logger(), "Fahrmodus: STRAIGHT - Fahre geradeaus mit %.2f m/s", 
                        current_linear_speed);
        }

        RCLCPP_DEBUG(this->get_logger(),
            "Zeit: %.3f, dt: %.4fs, LinSpd: %.2fm/s, Wheelbase: %.2fm, k_p: %.2f",
            current_time.seconds(), dt, linear_speed, wheelbase, k_p);
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
    
    rclcpp::Time last_time_;              // Zeitpunkt der letzten Kontrollschleife für dt-Berechnung
    double remaining_heading_change_;     // Verbleibende Kursänderung in Radiant
    DriveMode drive_mode_;                // Aktueller Fahrmodus
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ModelCarBicycleControlNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}