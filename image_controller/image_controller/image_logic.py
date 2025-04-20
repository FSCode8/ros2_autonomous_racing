#!/usr/bin/env python3

from image_controller.camera_software import CameraGeometry
from image_controller.camera_software import get_intrinsic_matrix

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image

import cv2 as cv
import numpy as np
from cv_bridge import CvBridge, CvBridgeError


def warp_image_with_homography(image, H, output_size=None):
    """
    Warp an image using a homography matrix
    
    Parameters:
    image (numpy.ndarray): Input image
    H (numpy.ndarray): Homography matrix (3x3)
    output_size (tuple): Size of output image (width, height)
    
    Returns:
    numpy.ndarray: Warped image
    """
    if output_size is None:
        output_size = (image.shape[1], image.shape[0])
    
    # Apply the homography transform
    warped_image = cv.warpPerspective(image, H, output_size)
    
    return warped_image


class ImageTransformer(Node):

    def __init__(self):
        super().__init__('image_transformer')
        # NOTE: Add server for camera_info?

        self.bridge = CvBridge()

        # Create TF buffer and listener
        #self.tf_buffer = tf2_ros.Buffer()
        #self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.camera_obj = CameraGeometry()

        # Create homography matrix
        self.H = self.camera_obj.create_homography_for_iso8855()

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
        transformed_image = warp_image_with_homography(cv_image, self.H)
        # Display image
        cv.imshow("Camera Feed", transformed_image)
        cv.waitKey(1)


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