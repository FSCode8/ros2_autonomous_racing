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

from nav_msgs.msg import OccupancyGrid
from std_msgs.msg  import Header
from geometry_msgs.msg import Pose

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
import os


def get_intrinsic_matrix():
    alpha = 476.7014503479004
    Cu = 400.0
    Cv = 400.0
    return np.array([[alpha, 0, Cu],
                     [0, alpha, Cv],
                     [0, 0, 1.0]])


class Test(Node):

    def __init__(self):
        super().__init__('image_transformer')
        # NOTE: Add server for camera_info?

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

        self.image_topic_name = '/image_raw'

        image_bag_flag = True
        full_setup_flag = False

        if image_bag_flag:

            if full_setup_flag:
                camera_obj = Camera(K_matrix=get_intrinsic_matrix())
                vehicle_obj = VehicleGeometry(cam_height=1, len_vehicle_shadow=2.24, len_vehicle_front=1.0)
            
                rotation_matrix = np.array([[0, 0, 1],
                            [-1, 0, 0],
                            [0, -1, 0]])

                translation_vector = np.array([0, 0, 1]) 

                self.vision_calc = VisionCalculation(
                    camera_object=camera_obj,
                    vehicle_object=vehicle_obj,
                    rotation_cam_to_world=rotation_matrix,
                    translation_cam_to_world=translation_vector
                )

                self.vision_calc.test_camera_geometry(test_vec_camframe=np.array([0,1,0]), test_vec_worldframe=np.array([0, 0, 1]))

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


    def _initial_data_callback(self, msg):
        
        self._intrinsic_matrix = np.array(msg.k).reshape(3, 3)
        self.get_logger().info(f'Received camera-intrinsic-matrix: {self._intrinsic_matrix}')
        
        self.destroy_subscription(self.subscription)
        self.get_logger().info('Subscription closed after receiving initial data')

        camera_obj = Camera(K_matrix=self._intrinsic_matrix)
        vehicle_obj = VehicleGeometry(cam_height=1, len_vehicle_shadow=2.24, len_vehicle_front=1.0)      
        
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

        self.vision_calc.test_camera_geometry(test_vec_camframe=np.array([0,1,0]), test_vec_worldframe=np.array([0, 0, 1]))

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
        
        # self.test(cv_image)
        # self.image_viewer(cv_image)
        self.image_saver(cv_image)
        return

    def test(self, image):
        # --- 1) Crop and edge-detect as before ---
        h, w = image.shape[:2]
        cx, cy = w // 2, h // 2
        x1, y1 = max(0, cx - 400), max(0, 445)
        x2, y2 = min(w, cx + 400), min(h, 550)
        roi = image[y1:y2, x1:x2]

        gray    = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (5, 5), 1.4)
        edges   = cv2.Canny(blurred, 150, 250)

        contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

        # --- 2) Define bottom-center and initialize trackers ---
        h_e, w_e = edges.shape
        target = np.array([w_e // 2, h_e - 1])

        print("Target:", target)

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
            print("Y max:", y_max_point)

            if y_max_point[0] < target[0]:
                left_dist = np.linalg.norm(y_max_point - target)
                print("Left dist:", left_dist)
                if left_dist < left_min:
                    left_min = left_dist
                    left_point = y_max_point
                    left_cnt = cnt
            else:
                right_dist = np.linalg.norm(y_max_point - target)
                if right_dist < right_min:
                    right_min = right_dist
                    right_point = y_max_point
                    right_cnt = cnt

        show_debug = True
        if show_debug:
            # --- 3.5) Draw the target point ---
            cv2.circle(roi, tuple(target), 5, (255, 0, 0), -1)
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
                x_left = 0
                x_right = w_e-1
            else:
                if left_point is None: 
                    print("Couldn't find a left contour.")
                    x_left = 0
                else:
                    xl, yl = left_cnt[:, 0, 0], left_cnt[:, 0, 1]
                    for xi, yi in zip(xl.astype(int), yl.astype(int)):
                        if 0 <= xi < w_e and 0 <= yi < h_e:
                            curve_img[yi, xi] = (0, 255, 0)

                    idx_left = np.argmin(np.abs(yl - y_target))     # Find closest index in yl and yr to y_target
                    x_left = xl[idx_left]   # Get corresponding x positions

                if right_point is None:
                    print("Couldn't find a right contour.")
                    x_right = w_e-1
                else: 
                    xr, yr = right_cnt[:, 0, 0], right_cnt[:, 0, 1]
                    for xi, yi in zip(xr.astype(int), yr.astype(int)):
                        if 0 <= xi < w_e and 0 <= yi < h_e:
                            curve_img[yi, xi] = (0, 255, 0)

                    idx_right = np.argmin(np.abs(yr - y_target))    # Find closest index in yl and yr to y_target
                    x_right = xr[idx_right]     # Get corresponding x positions

        # Create points
        pt1 = (int(x_left), int(y_target))
        pt2 = (int(x_right), int(y_target))

        # Draw the horizontal line in red
        cv2.line(curve_img, pt1, pt2, color=(0, 0, 255), thickness=2)
        
        cv2.imshow("Curves", curve_img)
        cv2.waitKey(0)
        return

    def image_viewer(self, image):
        # Display the image in a window
        cv2.imshow("Image", image)
        cv2.waitKey(1)

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


def start_occupancy_grid():
    # meters per pixel in your desired occupancy grid
    resolution = 0.05  

    # bounds of your grid in world coords
    x_min, x_max = -35.0, +35.0
    y_min, y_max = -35.0, +35.0

    # compute grid size
    width  = int((x_max - x_min) / resolution)
    height = int((y_max - y_min) / resolution)

    grid = OccupancyGrid()
    grid.header = Header(frame_id="map")
    grid.info.resolution = resolution
    grid.info.width  = width
    grid.info.height = height

    # origin of the grid in the world
    grid.info.origin = Pose()
    grid.info.origin.position.x = 3.0
    grid.info.origin.position.y = 4.0
    grid.info.origin.position.z = 0.0

    # start with unknown
    data = np.full((height, width), -1, dtype=np.int8)

    grid.data = data.flatten().tolist()  # flatten the 2D array to 1D list

    return grid

    # for each pixel in your probability map…
    for i in range(P.shape[0]):
        for j in range(P.shape[1]):
            prob = P[i, j]

            # compute world coords of this pixel
            x, y, z = pixel_to_world(i, j)   # your reprojection

            # compute grid indices
            gx = int((x - x_min) / resolution)
            gy = int((y - y_min) / resolution)

            # check bounds
            if 0 <= gx < width and 0 <= gy < height:
                # decide occupancy: e.g.
                if prob > 0.5:
                    # “lane” pixels as free
                    data[gy, gx] = int((1.0 - prob) * 99)  
                    # (higher prob → lower cost)
                else:
                    # non‑lane as occupied
                    data[gy, gx] = 100

    return grid

class LaneGridPublisher(Node):
    def __init__(self):
        super().__init__('lane_grid_pub')

        self.grid = start_occupancy_grid()

        self.pub = self.create_publisher(OccupancyGrid, '/occ_grid', 10)
        self.timer = self.create_timer(0.5, self.publish_grid)

    def publish_grid(self):
        # assume grid is already computed (above)
        self.grid.header.stamp = self.get_clock().now().to_msg()
        self.pub.publish(self.grid)

if __name__ == '__main__':

    single_image_flag = True
    grid_flag = True

    rclpy.init()
    test_node = Test()

    grid_node = LaneGridPublisher()

    if grid_flag:
        # Start the occupancy grid publisher
        rclpy.spin(grid_node)
        grid_node.destroy_node()
    else:
        if single_image_flag:
            image = cv2.imread('./manually_saved_images/saved_image_00006.png')  # Replace with your actual path

            # Check if the image was loaded successfully
            if image is None:
                print("Error: Could not read the image.")
            else:
                test_node.test(image)
        else:  
            rclpy.spin(test_node)
            test_node.destroy_node()

    rclpy.shutdown()