#!/usr/bin/env python3

from image_controller.camera_software import CameraGeometry
from image_controller.VisionCalculation import Camera
from image_controller.VisionCalculation import VehicleGeometry
from image_controller.VisionCalculation import VisionCalculation

import rclpy
from rclpy.node import Node
from rclpy.task import Future
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
from std_msgs.msg import Float32


from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from geometry_msgs.msg import PointStamped
import tf2_geometry_msgs

import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Vector3

import math


def get_intrinsic_matrix():
    alpha = 476.7014503479004
    Cu = 400.0
    Cv = 400.0
    return np.array([[alpha, 0, Cu],
                     [0, alpha, Cv],
                     [0, 0, 1.0]])


class ImageTransformer(Node):

    def __init__(self):
        super().__init__('image_transformer')
        # NOTE: Add server for camera_info?

        # Vehicle constants
        self.cam_height = 1.0
        self.len_vehicle_front = 1.0
        self.len_vehicle_shadow = 2.24
        self.image_height = 800
        self.image_width = 800

        self.image_topic_name = '/image_raw'

        image_bag = True # image bag or simulation/real car flag

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.bridge = CvBridge()

        # Publisher for transformed image
        self.publisher_ = self.create_publisher(
            Float32,
            '/collision_distance',
            10) 
        
        # publisher for next waypoint
        self.publisher_next_waypoint_ = self.create_publisher(
            Vector3,
            '/next_waypoint',
            10)

        if image_bag:
            camera_obj = Camera(K_matrix=get_intrinsic_matrix())
            vehicle_obj = VehicleGeometry(cam_height=self.cam_height , len_vehicle_shadow=self.len_vehicle_shadow, len_vehicle_front=self.len_vehicle_front)

            rotation_matrix = np.array([[0, 0, 1],
                        [-1, 0, 0],
                        [0, -1, 0]])

            translation_vector = np.array([0, 0, self.cam_height]) 

            self.vision_calc = VisionCalculation(
                camera_object=camera_obj,
                vehicle_object=vehicle_obj,
                rotation_cam_to_world=rotation_matrix,
                translation_cam_to_world=translation_vector
            )

            self.min_carless_pixel = int(self.vision_calc.get_min_carless_pixel()[1]) 
            
            self.subscription = self.create_subscription(
                Image,
                self.image_topic_name,
                self.execute_callback,
                10
            )

            self.get_logger().info("ImageTransformer node init finished.")

        else:
            # Create subscription with the callback group
            self.subscription = self.create_subscription(
                CameraInfo,  # Replace with your message type
                '/camera_info',
                self._initial_data_callback,
                10
            )

            self.get_logger().info("ImageTransformer node has been started.")

            # Wait for the initial data
            self.get_logger().info('Waiting for initial data from topic...')

    def _initial_data_callback(self, msg):
        
        self._intrinsic_matrix = np.array(msg.k).reshape(3, 3)
        self.get_logger().info(f'Received camera-intrinsic-matrix: {self._intrinsic_matrix}')
        
        self.destroy_subscription(self.subscription)
        self.get_logger().info('Subscription closed after receiving initial data')

        camera_obj = Camera(K_matrix=self._intrinsic_matrix)
        vehicle_obj = VehicleGeometry(cam_height=self.cam_height , len_vehicle_shadow=self.len_vehicle_shadow, len_vehicle_front=self.len_vehicle_front)      
        
        # get transformation from base_link to camera_link_optical
        from_frame_rel = 'camera_link_optical'
        to_frame_rel = 'base_link'

        try:
            t = self.tf_buffer.lookup_transform(
                to_frame_rel,
                from_frame_rel,
                rclpy.time.Time())
        except TransformException as ex:
            self.get_logger().warn(
                f'Could not transform {from_frame_rel} to {to_frame_rel}: {ex}')
            return     

        # Extract translation and rotation
        trans = t.transform.translation
        rot = t.transform.rotation
        
        # Point in camframe
        np_point = np.array([0.0, 0.0, 0.0])

        # Convert the quaternion to a rotation matrix
        rotation_matrix = VisionCalculation.quaternion_to_rotation_matrix(rot.x, rot.y, rot.z, rot.w)
        
        # Transform the point
        transformed_np_point = rotation_matrix @ np_point + np.array([trans.x, trans.y, trans.z])
        
        translation_vector = np.array([0, trans.y, trans.z])    # world frame origin is under camera, baselink is one behind

        self.vision_calc = VisionCalculation(
            camera_object=camera_obj,
            vehicle_object=vehicle_obj,
            rotation_cam_to_world=rotation_matrix,
            translation_cam_to_world=translation_vector
        )

        self.min_carless_pixel = int(self.vision_calc.get_min_carless_pixel()[1]) 
        
        self.subscription = self.create_subscription(
            Image,
            self.image_topic_name,
            self.execute_callback,
            10
        )

    def execute_callback(self, msg):
        # print(self.vision_calc.grid_coordinates[799, 400])
        
        self.get_logger().info('Receiving video frame')
        # Convert ROS Image message to OpenCV2 image
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except CvBridgeError as e:
            self.get_logger().error(f'CvBridge Error: {e}')

        #transformed_image = cv2.warpPerspective(image, H, (image.shape[1], image.shape[0]))
        #cv2.imwrite("/root/ros2_autonomous_racing/camera_view.png", cv_image)

        # compute minimum distance
        compute_min_distance = self.compute_min_distance(cv_image)
        self.get_logger().info(f'Minimum distance: {compute_min_distance}')
        msg = Float32()
        msg.data = compute_min_distance
        self.publisher_.publish(msg)

        #compute_next_waypoint = self.compute_next_waypoint(cv_image)
        
        cv2.imshow("Camera Feed", cv_image)
        cv2.waitKey(1)

    def compute_min_distance(self, image):
        # Convert the image to grayscale
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        # Threshold the image to create a binary image
        _, binary = cv2.threshold(gray, 190, 255, cv2.THRESH_BINARY)

        #binary[547, 0:800] = 255

        # Initialize minimum distance
        min_distance = float(-1)

        # Loop through each contour
        for v in range(self.min_carless_pixel, 400, -1):  # from min_carless_pixel (about 650) to 400 inclusive
            if binary[v, 400] == 255:    # (v, u) == (y, x)
                # Calculate the distance from the contour to the camera
                min_distance = self.vision_calc.compute_dist(400, v) # (u, v)
                
                return min_distance   

        return min_distance     

    def compute_next_waypoint(self, image):
        # Convert the image to grayscale
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        # Threshold the image to create a binary image
        _, binary = cv2.threshold(gray, 190, 255, cv2.THRESH_BINARY)

        # Find contours in the binary image
        # Image size (make sure it's large enough to contain the line)
        height, width = 800, 800
        mask = np.zeros((height, width), dtype=np.uint8)

        # Define start and end points
        pt1 = (10, 20)
        pt2 = (80, 90)

        # Draw line on mask
        cv2.line(mask, pt1, pt2, color=255, thickness=1)

        # Get coordinates of the pixels on the line
        line_pixels = np.column_stack(np.where(mask > 0))  # (row, col) = (y, x)

        # Convert to (x, y) format if needed
        line_pixels = [(x, y) for y, x in line_pixels]


if __name__ == '__main__':
    
    # Initialize
    rclpy.init(args=None)
    image_transformer = ImageTransformer()
    
    # Run
    rclpy.spin(image_transformer)

    # Shutdown
    image_transformer.destroy_node()
    rclpy.shutdown()