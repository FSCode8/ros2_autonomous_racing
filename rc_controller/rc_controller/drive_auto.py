import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
import time

class AutonomousCarDrive(Node):
    def __init__(self):
        super().__init__('autonomous_car')
        self.pub_ref = self.create_publisher(TwistStamped,'/bicycle_steering_controller/reference', 10)
        self.sub_slope = self.create_subscription(Float64,'/lane_detection/slope_value', self.get_slope, 10)
        self.sub = self.create_subscription(Odometry, '/bicycle_steering_controller/odometry', self.odom_callback, 10)
        self.start_position_x = None
        self.distance_traveled = 0.0
        #self.timer = self.create_timer(10, self.drive_forward)
        self.count = 0
        #self.time = time.clock()

    def get_slope(self, msg):
        slope = msg.data # neg values in list means path turning right, pos values path turning left, pos and neg values in list means straight
        print(f"slope is: {slope}") # Time: {self.time - time.clock()}")
        
        if slope > 0:
            self.drive_turn(False)
            print("Drive left")

        elif slope < 0:
            self.drive_turn()
            print("drıve rıght")
        else:
            self.drive_forward()  
            print("drive strıght")

    def odom_callback(self, msg):
        current_position_x = msg.pose.pose.position.x
        #print(f"current_position {current_position_x}")
        if self.start_position_x is None:
            self.start_position_x = current_position_x        
        
        # Calculate the distance traveled in the x-direction
        self.distance_traveled = current_position_x - self.start_position_x

        if self.distance_traveled >= 10.0:
            self.stop_car()

    def drive_forward(self):
        move_cmd = TwistStamped()
        move_cmd.twist.linear.x = 0.19 # Move forward at 1 m/s
        move_cmd.twist.angular.z = 0.0  # No steering

        if self.distance_traveled < 5.0:
            self.pub_ref.publish(move_cmd)
        else:
            self.stop_car()
    def drive_turn(self, left = True):
        move_cmd = TwistStamped()
        move_cmd.twist.linear.x = 0.19 # Move forward at 1 m/s
        if left:
            move_cmd.twist.angular.z = 0.145 #magnitude of steering to the left
        else:
            move_cmd.twist.angular.z = -0.145 #magnitude of steering to the left

        if self.distance_traveled < 5.0:
            self.pub_ref.publish(move_cmd)
        else:
            self.stop_car()

    def stop_car(self):
        stop_cmd = TwistStamped()
        stop_cmd.twist.linear.x = 0.0  # Stop moving
        stop_cmd.twist.angular.z = 0.0  # Ensure no steering
        self.pub.publish(stop_cmd)

def main(args=None):
    move_cmd = TwistStamped()
    move_cmd.twist.linear.x = 0.0 # Move forward at 1 m/s
    move_cmd.twist.angular.z = 0.0  # No steering
    rclpy.init(args=args)

    minimal_publisher = AutonomousCarDrive()

    rclpy.spin(minimal_publisher)


    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    
    main()