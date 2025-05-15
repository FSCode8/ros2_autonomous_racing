#!/usr/bin/env python3

import os
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2

class ImageSaverNode(Node):
    def __init__(self):
        super().__init__('image_saver_node')
        self.image_count = 0
        self.saved_count = 0
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(
            Image,
            '/image_raw',
            self.image_callback,
            10
        )
        self.get_logger().info("ImageSaverNode started, subscribing to '/image_raw'")

    def image_callback(self, msg):
        self.image_count += 1
        if self.image_count % 10 != 0:
            return

        try:
            #self.get_logger().info(f"Running from: {os.getcwd()}")
            try:
                cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            except CvBridgeError as e:
                self.get_logger().error(f'CvBridge Error: {e}')

            # Convert the image to grayscale
            gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

            # Threshold the image to create a binary image
            _, binary = cv2.threshold(gray, 190, 255, cv2.THRESH_BINARY)

            filename = f"./src/image_data/saved_image_{self.saved_count:05d}.png"
            os.makedirs(os.path.dirname(filename), exist_ok=True)
            if not cv2.imwrite(filename, cv_image):
                self.get_logger().error(f"Failed to save image: {filename}")
            else:
                self.get_logger().info(f"Saved {filename}")
                self.saved_count += 1
        except CvBridgeError as e:
            self.get_logger().error(f"cv_bridge exception: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = ImageSaverNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()