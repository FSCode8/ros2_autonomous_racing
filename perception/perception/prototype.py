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

from nav_msgs.msg import OccupancyGrid
from std_msgs.msg  import Header
from geometry_msgs.msg import Pose
from nav_msgs.msg import Path
from nav2_msgs.action import FollowPath

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PointStamped
import tf2_geometry_msgs

import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Vector3

import math
import os

import scipy


def get_intrinsic_matrix():
    alpha = 587.02559142
    Cu = 632.03759266
    Cv = 512.96361401
    return np.array([[alpha, 0, Cu],
                     [0, alpha, Cv],
                     [0, 0, 1.0]])


class Test(Node):

    def __init__(self):
        super().__init__('image_transformer')
        # NOTE: Add server for camera_info?

        # Vehicle constants
        self.cam_height = 0.23
        self.len_vehicle_front = 0.3
        self.len_vehicle_shadow = 0.4 # ?
        self.image_height = 960
        self.image_width = 1280

        self.image_topic_name = '/image_raw'#'/my_camera/pylon_ros2_camera_node/image_rect' 

        self.toggle_image_processing = True  # Toggle image processing on/off
        
        self.Driving_Stack_Existing = False  # Flag if there is the full simulation with the nav2 stack
        
        image_bag_flag = True
        full_setup_flag = True

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

        data = np.full((height, width), -1, dtype=np.int8) # All unknown
        self.grid.data = data.ravel().tolist()

        # Publisher for the occupancy grid
        self.grid_pub = self.create_publisher(OccupancyGrid, '/lane_grid', 10)        

        # Action client for FollowPath
        self.action_client = ActionClient(self, FollowPath, '/follow_path')
        
        #self.get_logger().info("FollowPath action server available!")

        if image_bag_flag:

            if full_setup_flag:
                camera_obj = Camera(K_matrix=get_intrinsic_matrix(), image_height=self.image_height, image_width=self.image_width)
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

                self.vision_calc.test_camera_geometry(test_vec_camframe=np.array([0,self.cam_height,0]), test_vec_worldframe=np.array([0, 0, self.cam_height]))

                self.min_carless_pixel = int(self.vision_calc.get_min_carless_pixel()[1]) 
            
            self.subscription = self.create_subscription(
                Image,
                self.image_topic_name,
                self.execute_callback,
                10
            )
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
        self.get_logger().info("Init finished")
        #self.action_client.wait_for_server()


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
        print(f"Quaternion: {rot.x}, {rot.y}, {rot.z}, {rot.w}")
        print(f"Translation: {trans.x}, {trans.y}, {trans.z}")
        rotation_matrix = VisionCalculation.quaternion_to_rotation_matrix(rot.x, rot.y, rot.z, rot.w)
        
        # Transform the point
        print(f"Rotation matrix: {rotation_matrix}")
        transformed_np_point = rotation_matrix @ np_point + np.array([trans.x, trans.y, trans.z])

        self.get_logger().info(f"Transformed point: {transformed_np_point}")
        
        translation_vector = np.array([0, trans.y, trans.z])    # world frame origin is under camera, baselink is one behind

        self.vision_calc = VisionCalculation(
            camera_object=camera_obj,
            vehicle_object=vehicle_obj,
            rotation_cam_to_world=rotation_matrix,
            translation_cam_to_world=translation_vector
        )

        self.vision_calc.test_camera_geometry(test_vec_camframe=np.array([0,self.cam_height,0]), test_vec_worldframe=np.array([0, 0, self.cam_height]))

        print("POINT:")
        print(self.vision_calc.grid_coordinates[799, 400])
        print(self.vision_calc.grid_coordinates[401, 400])
        print(self.vision_calc.grid_coordinates[0, 400])

        self.min_carless_pixel = int(self.vision_calc.get_min_carless_pixel()[1]) 
        
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
        #self.test(cv_image)
        #self.image_viewer(cv_image)
        self.image_saver(cv_image)
        return

    def detect_lanes(self, image):
        print("Detecting lanes...")
        # --- 1) Crop and edge-detect as before ---
        h, w = image.shape[:2]
        cx, cy = w // 2, h // 2
        x1, y1 = 0, h // 2 
        x2, y2 = w, h 
        roi = image[y1:y2, x1:x2]

        h_e, w_e, _ = roi.shape

        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (5, 5), 1.4)
        edges = cv2.Canny(blurred, 150, 200)

        cv2.rectangle(edges, ((w_e//2)-(w_e//4), h_e-20), ((w_e//2)+(w_e//4), h_e-1), (0,0,0), -1)

        contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

        # --- 2) Define bottom-center and initialize trackers ---
        h_e, w_e = edges.shape
        target = np.array([w_e // 2, h_e - 1])
        target_l = np.array([target[0]-((target[0]//4)*3), target[1]-50])
        target_r = np.array([target[0]+((target[0]//4)*3), target[1]-50])

        left_min = float('inf')
        right_min = float('inf')
        left_cnt, left_point = None, None
        right_cnt, right_point = None, None

        # --- 3) For each contour, find its closest point to target and bucket by side ---
        y_max_point_array = []
        for cnt in contours:
            idx = np.argmax(cnt[:, 0, 1])
            y_max_point = cnt[idx, 0, :]
            y_max_point_array.append(y_max_point)

            if len(cnt)>100 and y_max_point[1]>target[1]//2: # filter out small or wrongcontours
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

        #print("len cnt l:", len(left_cnt))
        #print("len cnt r:", len(right_cnt))
        show_debug = False
        if show_debug:
            # --- 3.5) Draw the target point ---
            cv2.circle(roi, tuple(target), 5, (255, 0, 0), -1)
            cv2.circle(roi, tuple(target_l), 5, (255, 0, 0), -1)
            cv2.circle(roi, tuple(target_r), 5, (255, 0, 0), -1)

            cv2.circle(roi, (target[0], target[1]//2), 5, (0, 0, 255), -1)
            cv2.imshow("Target", roi)
            cv2.waitKey(0)
            # --- 3.5) Draw the closest points ---
            print("Left closest point:", left_point)
            print("Right closest point:", right_point)
            if left_cnt is not None:
                cv2.circle(roi, tuple(left_point), 5, (0, 255, 0), -1)
            if right_cnt is not None:
                cv2.circle(roi, tuple(right_point), 5, (0, 0, 255), -1)
            cv2.imshow("Closest Points", roi)
            cv2.waitKey(0)

            contours_img = cv2.cvtColor(edges, cv2.COLOR_GRAY2BGR)
            if left_cnt is not None:
                cv2.drawContours(contours_img, [left_cnt], -1, (0,255,0), thickness=2)
            if right_cnt is not None:
                cv2.drawContours(contours_img, [right_cnt], -1, (0,0,255), thickness=2)

            cv2.imshow("Contours", contours_img)
            cv2.waitKey(0)

        # --- 5) Draw the contours ---
        y_target = 50  # must be within range(yl.min(), yl.max())

        fitting_flag = False
        if fitting_flag:
            # --- 4) Curve-fitting helper ---
            def fit_curve(cnt, deg=7, n_pts=500):
                x = cnt[:, 0, 0]
                y = cnt[:, 0, 1]
                coeffs = np.polyfit(x, y, deg)
                poly   = np.poly1d(coeffs)
                xp     = np.linspace(x.min(), x.max(), n_pts)
                yp     = poly(xp)
                return xp, yp

            # Sides logic
            curve_img = cv2.cvtColor(edges, cv2.COLOR_GRAY2BGR)
            if left_point is None and right_point is None:
                print("Couldn't find both a left and right contour.")
            else:
                if left_point is None: 
                    print("Couldn't find a left contour.")
                else:
                    xl, yl = fit_curve(left_cnt)    # fit left curve
                    
                    for xi, yi in zip(xl.astype(int), yl.astype(int)):
                        if 0 <= xi < w_e and 0 <= yi < h_e:
                            curve_img[yi, xi] = (0, 255, 0)

                if right_point is None:
                    print("Couldn't find a right contour.")   
                else:
                    xr, yr = fit_curve(right_cnt)   # fit right curve

                    for xi, yi in zip(xr.astype(int), yr.astype(int)):
                        if 0 <= xi < w_e and 0 <= yi < h_e:
                            curve_img[yi, xi] = (0, 255, 0)
        else:
            # Sides logic
            curve_img = cv2.cvtColor(edges, cv2.COLOR_GRAY2BGR)
            if left_point is None and right_point is None:
                print("Couldn't find both a left and right contour.")
                x_left = -1
                x_right = -1
            else:
                if left_point is None: 
                    print("Couldn't find a left contour.")
                    x_left = -1
                else:
                    xl, yl = left_cnt[:, 0, 0], left_cnt[:, 0, 1]
                    #for xi, yi in zip(xl.astype(int), yl.astype(int)):
                    #    if 0 <= xi < w_e and 0 <= yi < h_e:
                    #        curve_img[yi, xi] = (0, 255, 0)

                    idx_left = np.argmin(np.abs(yl - target[1]))     # Find closest index in yl and yr to y_target
                    x_left = xl[idx_left]   # Get corresponding x positions

                if right_point is None:
                    print("Couldn't find a right contour.")
                    x_right = -1#w_e-1
                else: 
                    xr, yr = right_cnt[:, 0, 0], right_cnt[:, 0, 1]
                    #for xi, yi in zip(xr.astype(int), yr.astype(int)):
                    #    if 0 <= xi < w_e and 0 <= yi < h_e:
                    #        curve_img[yi, xi] = (0, 255, 0)

                    idx_right = np.argmin(np.abs(yr - target[1]))    # Find closest index in yl and yr to y_target
                    x_right = xr[idx_right]     # Get corresponding x positions

        # Create points
        current_position = [0.0, 0.0, 0.0]  # X, Y, Z in world frame

        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = "odom"

        # Create a simple curved path
        path_msg.poses = []
        for i in range(7):
            pose = PoseStamped()
            pose.header.stamp = path_msg.header.stamp
            pose.header.frame_id = "odom"

            pixel_y = h_e - (i+1) * 60  

            # TODO: Add Logic for the case that left/right boundary goes until right/left side of the image -> point should be outside of the image
            # NOTE: Rotation is not considered yet 

            if x_left == -1 and x_right == -1:
                pixel_x = int(w_e / 2.0)  # Default to center if both sides are missing
            elif x_left == -1:
                pixel_x = 0
            elif x_right == -1:
                pixel_x = int(w_e - 1)

            else:
                idx_left = np.argmin(np.abs(yl - pixel_y))     # Find closest index in yl to y_target
                x_left = xl[idx_left]   # Get corresponding x positions

                idx_right = np.argmin(np.abs(yr - pixel_y))    # Find closest index in yr to y_target
                x_right = xr[idx_right]     # Get corresponding x positions

                pixel_x = int((x_left + x_right) / 2.0)
            
            #cv2.circle(curve_img, (pixel_x, pixel_y), 3, (0, 0, 255), -1)

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

        #cv2.imshow("Contours", curve_img)
        #cv2.waitKey(0)

        # Publisher for the occupancy grid
        self.publish_grid()

        goal_msg = FollowPath.Goal()
        goal_msg.path = path_msg

        if self.Driving_Stack_Existing:
            self.get_logger().info("Sending path to FollowPath action server.")
            self.action_client.send_goal_async(goal_msg).add_done_callback(self.goal_response_callback)
        else:
            self.get_logger().info("Path would be send to FollowPath action server.")

    def image_viewer(self, image):
        # Display the image in a window
        
        #h, w = image.shape[:2]

        #cv2.circle(image, (w//2, h//2), 5, (0, 0, 255), -1)
        # show
        h_e, w_e, _ = image.shape
        ("distances:", h_e, w_e)
        cv2.rectangle(image, ((w_e//2)-(w_e//4), h_e-20), ((w_e//2)+(w_e//4), h_e-1), (0,0,0), -1)

        #cv2.imshow("Image", binary)
        cv2.imshow("Image Original", image)

        cv2.waitKey(0)

    def image_saver(self, image):

        # Display the image
        cv2.imshow("Image", image)

        key = cv2.waitKey(0) & 0xFF  # Mask to handle cross-platform issues

        if key == ord('s'):
            # Save the image
            save_path = './manually_saved_images'
            os.makedirs(save_path, exist_ok=True)
            num_items = len(os.listdir(save_path))
            filename = os.path.join(save_path, f"saved_image_{num_items:05d}.png")

            if not cv2.imwrite(filename, image):
                self.get_logger().error(f"Failed to save image: {filename}")
            else:
                self.get_logger().info(f"Saved {filename}")
                
        else:
            print("Image not saved.")

    def publish_grid(self, update_information=None):
        occupancy_grid = OccupancyGrid()
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

    def warp_image(self, image):
        bev_img_size_px = (1280, 960)

        self.vision_calc.camera.K_matrix

        print(self.vision_calc.translation_cam_to_world)

        """
        H_bev = self.get_bev_homography(
            K_orig=self.vision_calc.cam_intrinsic_matrix,
            R_cw=self.vision_calc.rotation_cam_to_world,
            t_cam_in_world=self.vision_calc.translation_cam_to_world,
            world_roi_x_m=(-1, 1),
            world_roi_y_m=(0, 2),
            bev_img_size_px=(1280, 960),
            ground_plane_world_z=0.0,
            invert_bev_y_axis=True
            )
        """

        bev_image = cv2.warpPerspective(image, self.vision_calc.bev_homography, bev_img_size_px)

        cv2.imshow("Original Image", image)
        cv2.imshow("Bird's-Eye View Image", bev_image)
        cv2.waitKey(0)
        cv2.destroyAllWindows()

        pass

    
    def test(self, image):
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (5, 5), 1.4)
        edges = cv2.Canny(blurred, 100, 170) # Adjust thresholds

        height, width = image.shape[:2]
        roi_vertices = np.array([[(width / 2, height-20), (0, height), (0, height/(5)*4), (width / 2, height / 2 + 50), (width, height/(5)*4), (width, height)]], dtype=np.int32) # Adjust
        mask = np.zeros_like(edges) # or edges
        cv2.fillPoly(mask, roi_vertices, 255)
        masked_lanes = cv2.bitwise_and(edges, mask)

        #new_warp = cv2.warpPerspective(image, self.vision_calc.bev_homography, (image.shape[1], image.shape[0]))

        from camera_software import CameraGeometry 
        camera_geometry = CameraGeometry(height=self.cam_height, K_matrix=get_intrinsic_matrix())

       # https://docs.opencv.org/4.x/d9/dab/tutorial_homography.html

        R0c1 = camera_geometry.rotation_perspective_cam
        R0c2 = camera_geometry.rotation_perspective_bird

        R1 = R0c2 @ R0c1.T

        t1 = np.array([0, 0, 0]) # translation vector between camera and bird perspective

        n = np.array([0, 0, -1])

        d = self.cam_height

        H1 = R1 - (t1 * n.T / d)

        G = camera_geometry.intrinsic_matrix @ H1 @ np.linalg.inv(camera_geometry.intrinsic_matrix)

        img1_warp = cv2.warpPerspective(image, G, (image.shape[1], image.shape[0]))

        print("Camera Geometry:", camera_geometry.rotation_cam_to_world)
        print("Bird rotation", camera_geometry.rotation_perspective_bird)
        cv2.imshow("Original Image", image)
        cv2.waitKey(0)
        cv2.imshow("edges", img1_warp)
        cv2.waitKey(0)
        cv2.destroyAllWindows()

        return
        cv2.imshow("Original Image", image)
        cv2.waitKey(0)
        cv2.imshow("edges", edges)
        cv2.waitKey(0)
        cv2.imshow("warp", new_warp)
        cv2.waitKey(0)
        cv2.imshow("Masked Lanes", masked_lanes)
        cv2.waitKey(0)
        cv2.destroyAllWindows()

"""np.array([[0, 0, 1],
            [-1, 0, 0],
            [0, -1, 0]])"""

if __name__ == '__main__':

    single_image_flag = True
    grid_flag = False
    test_timing_flag = True

    rclpy.init()

    if test_timing_flag:
        import time

        test_node = Test()
        print("After init")
        image = cv2.imread('./saved_clean_track_images/image_1749133787_843306938.png') 

        start_time = time.perf_counter()
        test_node.detect_lanes(image)
        end_time = time.perf_counter()
        execution_time = end_time - start_time
        print(f"Execution time: {execution_time:.6f} seconds")
        
    else:
        test_node = Test()
        test_node.get_logger().info("Test node has been started.")
        if single_image_flag:
            #image = cv2.imread('./saved_images/image_1747926141_543872257.png')  
            #image = cv2.imread('./saved_images/image_1747926103_906651227.png')  
            
            image = cv2.imread('./saved_clean_track_images/image_1749133787_843306938.png') 
            #image = cv2.imread('./manually_saved_images/saved_image_00012.png')
            # Check if the image was loaded successfully
            if image is None:
                print("Error: Could not read the image.")
            else:
                #from image_controller.run_through_script import set_pose
                #set_pose("ackermann", 5.0, 0.0, 0.0)
                test_node.detect_lanes(image)
        else:  
            rclpy.spin(test_node)
            test_node.destroy_node()

    rclpy.shutdown()