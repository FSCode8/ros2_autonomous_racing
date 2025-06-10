#!/usr/bin/env python3

from image_controller.camera_software import CameraGeometry
from image_controller.VisionCalculation import Camera
from image_controller.VisionCalculation import VehicleGeometry
from image_controller.VisionCalculation import VisionCalculation

import rclpy
from rclpy.node import Node
from rclpy.task import Future
from rclpy.action import ActionClient
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

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


class ImageTransformer(Node):

    def __init__(self):
        super().__init__('image_transformer')

        # Vehicle constants
        self.cam_height = 0.23
        self.len_vehicle_front = 0.3        
        self.len_vehicle_shadow = 0.2
        self.image_height = 960
        self.image_width = 1280

        self.image_topic_name = '/image_raw'
        self.toggle_image_processing = True  # Toggle image processing on/off 

        self.Driving_Stack_Existing = True  # Flag if there is the full simulation with the nav2 stack 

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.bridge = CvBridge()

        # Publisher for transformed image
        self.publisher_ = self.create_publisher(
            Float32,
            '/collision_distance',
            10) 
        
        # Create occupancy grid
        resolution = 0.01
        x_min, x_max = -15.0, +15.0
        y_min, y_max = -15.0, +15.0
        width  = int((x_max - x_min) / resolution)
        height = int((y_max - y_min) / resolution)

        self.grid = OccupancyGrid()
        self.grid.header = Header()
        self.grid.header.frame_id = "odom"
        self.grid.info.map_load_time = self.get_clock().now().to_msg()
        self.grid.info.resolution = resolution
        self.grid.info.width  = width
        self.grid.info.height = height
        self.grid.info.origin.position.x = 0.0
        self.grid.info.origin.position.y = 0.0
        self.grid.info.origin.position.z = 0.0

        data = np.full((height, width), 0, dtype=np.int8) # All unknown
        self.grid.data = data.ravel().tolist()

        # Occupancy grid publisher
        self.grid_pub = self.create_publisher(OccupancyGrid, '/lane_grid', 10)

        # Action client for FollowPath
        if self.Driving_Stack_Existing:
            self.action_client = ActionClient(self, FollowPath, '/follow_path')

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

        camera_obj = Camera(K_matrix=self._intrinsic_matrix, image_height=self.image_height, image_width=self.image_width)
        vehicle_obj = VehicleGeometry(cam_height=self.cam_height, len_vehicle_shadow=self.len_vehicle_shadow, len_vehicle_front=self.len_vehicle_front)      
        
        # get transformation from base_link to camera_link_optical
        from_frame_rel = 'camera_link_optical'
        to_frame_rel = 'base_link'
        for i in range(3):
            # wait and retry
            time.sleep(1)
            try:
                t = self.tf_buffer.lookup_transform(
                    to_frame_rel,
                    from_frame_rel,
                    rclpy.time.Time())
            except TransformException as ex:
                self.get_logger().warn(
                    f'Could not transform {from_frame_rel} to {to_frame_rel}: {ex}')
                if i == 2:    
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

        if self.Driving_Stack_Existing:
            self.get_logger().info(f'Wait for Action Server.')
            self.action_client.wait_for_server()
            self.get_logger().info(f'Action Server found.')
        
        self.subscription = self.create_subscription(
            Image,
            self.image_topic_name,
            self.execute_callback,
            10
        )

    def execute_callback(self, msg):
        
        self.get_logger().info('Receiving video frame')

        # Convert ROS Image message to OpenCV2 image
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except CvBridgeError as e:
            self.get_logger().error(f'CvBridge Error: {e}')

        # compute minimum distance
        if False:   # Not in use
            compute_min_distance = self.compute_min_distance(cv_image)
            self.get_logger().info(f'Minimum distance: {compute_min_distance}')
            msg = Float32()
            msg.data = compute_min_distance
            self.publisher_.publish(msg)

        # send path and occupancy grid out of image
        if self.toggle_image_processing:
            self.send_local_path(cv_image)
            self.toggle_image_processing = False
        else:
            self.toggle_image_processing = True

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

    def send_local_path(self, image):
        # region of interest is one pixel under the middle for pitch angle = 0  
        h, w = image.shape[:2]
        cx, cy = w // 2, h // 2
        x1, y1 = 0, h // 2 
        x2, y2 = w, h 
        roi = image[y1:y2, x1:x2]

        h_e, w_e, _ = roi.shape

        # convert image, apply the Canny Edge Detection and find the contours to get the lane markings
        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (5, 5), 1.4)
        edges = cv2.Canny(blurred, 150, 200)

        # paint over car part
        cv2.rectangle(edges, ((w_e//2)-(w_e//4), h_e-20), ((w_e//2)+(w_e//4), h_e-1), (0,0,0), -1)

        contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

        # Find the contours which start closest to the bottom middle of the image
        # They are most likely to be the lane markings of interest
        h_e, w_e = edges.shape
        target = np.array([w_e // 2, h_e - 1])
        target_l = np.array([target[0]-((target[0]//4)*3), target[1]-50])
        target_r = np.array([target[0]+((target[0]//4)*3), target[1]-50])

        left_min = float('inf')
        right_min = float('inf')
        left_cnt, left_point = None, None
        right_cnt, right_point = None, None

        y_max_point_array = []
        for cnt in contours:
            idx = np.argmax(cnt[:, 0, 1])
            y_max_point = cnt[idx, 0, :]
            y_max_point_array.append(y_max_point)

            if len(cnt)>100 and y_max_point[1]>target[1]//3: # filter out small or wrongcontours
                if y_max_point[0] < target[0]:
                    left_dist = np.linalg.norm(target_l-y_max_point)
                    if left_dist < left_min:
                        left_min = left_dist
                        left_point = y_max_point
                        left_cnt = cnt
                else:
                    right_dist = np.linalg.norm(target_r-y_max_point)
                    if right_dist < right_min:
                        right_min = right_dist
                        right_point = y_max_point
                        right_cnt = cnt

        # setup the arrays with the x values of the left and right lane
        if left_point is None and right_point is None:
            x_left = -1
            x_right = -1
        else:
            if left_point is None: 
                x_left = -1
            else:
                xl, yl = left_cnt[:, 0, 0], left_cnt[:, 0, 1]

                idx_left = np.argmin(np.abs(yl - target[1]))     # Find closest index in yl to target y
                x_left = xl[idx_left]   # Get corresponding x positions

            if right_point is None:
                x_right = -1
            else: 
                xr, yr = right_cnt[:, 0, 0], right_cnt[:, 0, 1]

                idx_right = np.argmin(np.abs(yr - target[1]))    # Find closest index in yr to target y
                x_right = xr[idx_right]     # Get corresponding x positions

        # Create points
        current_position = [0.0, 0.0, 0.0]  # X, Y, Z in world frame

        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = "odom"

        contours_img = cv2.cvtColor(edges, cv2.COLOR_GRAY2BGR)
        if left_cnt is not None:
            cv2.drawContours(contours_img, [left_cnt], -1, (0,255,0), thickness=2)
        if right_cnt is not None:
            cv2.drawContours(contours_img, [right_cnt], -1, (0,0,255), thickness=2)

        # Create the path
        path_msg.poses = []
        only_one = False
        for i in range(6):
            pose = PoseStamped()
            pose.header.stamp = path_msg.header.stamp
            pose.header.frame_id = "odom"

            pixel_y = h_e - (i+2) * 60  

            # NOTE: Rotation is not considered yet 

            if x_left == -1 and x_right == -1:
                pixel_y = h_e // 2 
                pixel_x = int(w_e / 2.0)  # Default to center if both sides are missing
            elif x_left == -1:
                pixel_y = h_e // 2
                pixel_x = 0 + 120
            elif x_right == -1:
                pixel_y = h_e // 2
                pixel_x = int(w_e - 1) - 120
            else:
                idx_left = np.argmin(np.abs(yl - pixel_y))     # Find closest index in yl to y_target
                x_left = xl[idx_left]   # Get corresponding x positions

                idx_right = np.argmin(np.abs(yr - pixel_y))    # Find closest index in yr to y_target
                x_right = xr[idx_right]     # Get corresponding x positions

                pixel_x = int((x_left + x_right) / 2.0)

            cv2.circle(contours_img, (pixel_x, pixel_y), 5, (0, 255, 0), -1)

            pose.pose.position.x = current_position[0] + self.vision_calc.grid_coordinates[y1 + pixel_y, x1 + pixel_x][0]  # x coordinate in world frame
            pose.pose.position.y = current_position[1] + self.vision_calc.grid_coordinates[y1 + pixel_y, x1 + pixel_x][1]  # y coordinate in world frame 
            pose.pose.position.z = current_position[2]  # z coordinate in world frame
            
            if i> 0:
                dx = pose.pose.position.x - path_msg.poses[i-1].pose.position.x
                dy = pose.pose.position.y - path_msg.poses[i-1].pose.position.y
                  
            else:
                dx = pose.pose.position.x - current_position[0]
                dy = pose.pose.position.y - current_position[1]
            
            # Calculate yaw angle
            yaw = math.atan2(dy, dx)

            pose.pose.orientation.w = yaw  # No rotation
            path_msg.poses.append(pose)

            if only_one:
                break
        
        cv2.imshow("Contours", contours_img)
        cv2.waitKey(1)

        # Publisher for the occupancy grid
        self.publish_grid()

        goal_msg = FollowPath.Goal()
        goal_msg.path = path_msg

        if self.Driving_Stack_Existing:
            self.get_logger().info("Sending path to FollowPath action server.")
            self.action_client.send_goal_async(goal_msg).add_done_callback(self.goal_response_callback)
        else:
            self.get_logger().info("Path would be send to FollowPath action server.")
        

    def publish_grid(self, update_information=None):
        
        if update_information is None:
            self.grid.header.stamp = self.get_clock().now().to_msg()
            self.grid_pub.publish(self.grid)
        else:
            raise NotImplementedError("Grid update functionality not yet implemented")
        
        self.get_logger().info("Published occupancy grid to /lane_grid")

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


if __name__ == '__main__':
    keyboard_interrupt = False

    # Initialize
    rclpy.init(args=None)
    image_transformer = ImageTransformer()
    
    # Run
    try:
        image_transformer = ImageTransformer()
        rclpy.spin(image_transformer)
    except KeyboardInterrupt:
        pass  # Handle Ctrl+C gracefully
    finally:
        # Clean up
        if 'image_transformer' in locals():
            image_transformer.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
