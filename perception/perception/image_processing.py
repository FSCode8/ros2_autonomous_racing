#!/usr/bin/env python3

from perception.VisionCalculation import *
from perception.LaneDetection import LaneDetector

import rclpy
from rclpy.node import Node
from rclpy.task import Future
from rclpy.action import ActionClient

from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
from std_msgs.msg import Float32

from std_msgs.msg  import Header
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Pose
from nav_msgs.msg import Path
from nav2_msgs.action import FollowPath

from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PointStamped

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
import time


class ImageProcessor(Node):

    def __init__(self):
        super().__init__('image_transformer')

        # Vehicle constants
        self.cam_height = 0.23
        self.len_vehicle_front = 0.3        
        self.len_vehicle_shadow = 0.2
        self.image_height = 960
        self.image_width = 1280

        self.image_topic_name = '/image_raw'
        #self.image_topic_name = '/my_camera/pylon_ros2_camera_node/image_rect'

        self.Driving_Stack_Existing = False  # Flag if there is the full simulation with the nav2 stack 

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.bridge = CvBridge()
        
        # Occupancy grid update publisher
        self.grid_update_pub = self.create_publisher(Image, '/occ_grid_update', 10)

        # Action client for FollowPath
        if self.Driving_Stack_Existing:
            self.action_client = ActionClient(self, FollowPath, '/follow_path')
        
        # Set K-Matrix
        alpha = 587.02559142
        Cu = 632.03759266
        Cv = 512.96361401
        self._intrinsic_matrix =  np.array([[alpha, 0, Cu],
                                            [0, alpha, Cv],
                                            [0, 0, 1.0]])
        self.get_logger().info(f'Camera-intrinsic-matrix: \n{self._intrinsic_matrix}')

        camera_obj = Camera(K_matrix=self._intrinsic_matrix, image_height=self.image_height, image_width=self.image_width)
        vehicle_obj = VehicleGeometry(cam_height=self.cam_height, len_vehicle_shadow=self.len_vehicle_shadow, len_vehicle_front=self.len_vehicle_front)      
        
        # Set Rotation-Matrix cam_to_world
        rotation_matrix = np.array([[0, 0, 1],
                                    [-1, 0, 0],
                                    [0, -1, 0]])

        # rotation_matrix = rotation_matrix @ VisionCalculation.create_rotation_matrix(roll_deg=-5, pitch_deg=0, yaw_deg=0))

        translation_vector = np.array([0, 0, self.cam_height]) 

        self.vision_calc = VisionCalculation(
            camera_object=camera_obj,
            vehicle_object=vehicle_obj,
            rotation_cam_to_world=rotation_matrix,
            translation_cam_to_world=translation_vector
        )

        self.lane_detector = LaneDetector(vision_obj=self.vision_calc, model_path='./ft44_model.pth')
        self.get_logger().info(f'Lane Detector is using {self.lane_detector.device}')

        if self.Driving_Stack_Existing:
            self.get_logger().info(f'Wait for Action Server.')
            self.action_client.wait_for_server()
            self.get_logger().info(f'Action Server found.')
        
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1  # Only keep the most recent message
        )

        self.subscription = self.create_subscription(
            Image,
            self.image_topic_name,
            self.execute_callback,
            qos_profile
        )

        self.get_logger().info("ImageTransformer node has been started.")

    def execute_callback(self, msg):
        
        self.get_logger().info('Receiving video frame')

        # Convert ROS Image message to OpenCV2 image
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except CvBridgeError as e:
            self.get_logger().error(f'CvBridge Error: {e}')

        self.send_local_path(cv_image)
   

    def send_local_path(self, image):

        # path_msg = self.lane_detector.create_path_oldschool(image=image, time_stamp=self.get_clock().now().to_msg())

        path_msg = self.lane_detector.create_path(image=image, time_stamp=self.get_clock().now().to_msg())

        goal_msg = FollowPath.Goal()
        goal_msg.path = path_msg

        self.get_logger().info(f"path_msg: {path_msg.poses[2].pose}")

        if self.Driving_Stack_Existing:
            self.get_logger().info("Sending path to FollowPath action server.")
            self.action_client.send_goal_async(goal_msg).add_done_callback(self.goal_response_callback)
        else:
            self.get_logger().info("Path would be send to FollowPath action server.")
        

    def publish_detected_information(self, update_information):
        self.get_logger().info("[To be implemented] Publish occupancy grid update to /occ_grid_update")

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('FollowPath goal rejected')
            return
        self.get_logger().info('FollowPath goal accepted')
        goal_handle.get_result_async().add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        status = future.result().status
        self.get_logger().info(f"FollowPath action finished with status: {status}")