#!/usr/bin/env python3

from image_controller.camera_software import CameraGeometry, CameraGeometry2

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image

import cv2 as cv
import numpy as np
from cv_bridge import CvBridge, CvBridgeError


def get_intrinsic_matrix():
    alpha = 476.7014503479004
    Cu = 400.0
    Cv = 0.0
    return np.array([[alpha, 0, Cu],
                     [0, alpha, Cv],
                     [0, 0, 1.0]])

class ImageTransformer(Node):

    def __init__(self):
        super().__init__('image_transformer')
        # NOTE: Add server for camera_info?

        #self.bridge = CvBridge()

        # Create TF buffer and listener
        #self.tf_buffer = tf2_ros.Buffer()
        #self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.camera_obj = CameraGeometry(K_matrix=get_intrinsic_matrix())

        # Create homography matrix
        K = self.camera_obj.intrinsic_matrix
        R_t1 = self.camera_obj.trafo_perspective_cam
        R_t2 = self.camera_obj.trafo_perspective_bird
        self.H = self.camera_obj.create_perspective_homography(K1=K, K2=K, R_t1=R_t1, R_t2=R_t2)
        """
        self.subscription = self.create_subscription(
            Image,
            '/image_raw',
            self.transform_callback,
            10)
        
        # Publisher for transformed image
        self.publisher = self.create_publisher(
            Image,
            '/camera/iso8855_view',
            10) # prevent unused variable warning
        """
        self.test()

    def test(self):
        # Test the homography matrix
        image = cv.imread("/root/ros2_autonomous_racing/src/image_controller/camera_view.png")
        if image is None:
            self.get_logger().error("Failed to load image")
            return

        # Apply the homography transform
        transformed_image = cv.warpPerspective(image, self.H, (image.shape[1], image.shape[0]))

        # Display the original and transformed images
        cv.imshow("Original Image", image)
        cv.imshow("Transformed Image", transformed_image)
        cv.waitKey(0)
        cv.destroyAllWindows()
        
        rclpy.shutdown()

    def transform_callback(self, msg):
        self.get_logger().info('Receiving video frame')
        # Convert ROS Image message to OpenCV2 image
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        """
        try:
            cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except CvBridgeError as e:
            self.get_logger().error(f'CvBridge Error: {e}')
        """
        transformed_image = cv.warpPerspective(image, H, (image.shape[1], image.shape[0]))
        cv.imwrite("/root/ros2_autonomous_racing/camera_view.png", cv_image)
        # Display image
        cv.imshow("Camera Feed", cv_image)
        cv.waitKey(0)


def main(args=None):
    rclpy.init(args=args)

    image_transformer = ImageTransformer()

    rclpy.spin(image_transformer)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    image_transformer.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()