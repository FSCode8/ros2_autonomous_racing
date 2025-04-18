import image_controller

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image

import cv2 as cv
import numpy as np
from cv_bridge import CvBridge, CvBridgeError


class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        # NOTE: Add server for camera_info?
        self.subscription = self.create_subscription(
            Image,
            '\image_raw',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info('Receiving video frame')
        # Convert ROS Image message to OpenCV2
        bridge = CvBridge()
        try:
            cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except CvBridgeError as e:
            self.get_logger().error(f'CvBridge Error: {e}')

        # Display image
        cv.imshow("Camera Feed", cv_image)
        cv.waitKey(1)


def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()