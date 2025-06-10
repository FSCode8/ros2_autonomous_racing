import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry

class AutonomousCar(Node):
    def __init__(self):
        super().__init__('autonomous_car')
        self.pub = self.create_publisher(TwistStamped,'/bicycle_steering_controller/reference', 10)
        self.sub = self.create_subscription(Odometry, '/bicycle_steering_controller/odometry', self.odom_callback, 10)
        self.start_position_x = None
        self.distance_traveled = 0.0
        self.timer = self.create_timer(10, self.drive_forward)
        self.count = 0.0

    def odom_callback(self, msg):
        current_position_x = msg.pose.pose.position.x
        print(f"current_position {current_position_x}")
        if self.start_position_x is None:
            self.start_position_x = current_position_x

        self.drive_forward()
        # Calculate the distance traveled in the x-direction
        self.distance_traveled = current_position_x - self.start_position_x

        if self.distance_traveled >= 10.0:
            self.stop_car()

    def drive_forward(self):
        move_cmd = TwistStamped()
        move_cmd.twist.linear.x = 0.2  # Move forward at 1 m/s
        move_cmd.twist.angular.z = self.count  # No steering
        self.count += 10.1

        if self.distance_traveled < 10.0:
            self.pub.publish(move_cmd)
        else:
            self.stop_car()

    def stop_car(self):
        stop_cmd = TwistStamped()
        stop_cmd.twist.linear.x = 0.0  # Stop moving
        stop_cmd.twist.angular.z = 0.0  # Ensure no steering
        self.pub.publish(stop_cmd)

def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = AutonomousCar()

    rclpy.spin(minimal_publisher)


    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    
    main()