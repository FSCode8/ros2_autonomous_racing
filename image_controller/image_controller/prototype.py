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
        
        self.test(cv_image)
        return

    def test(self, image):
        # --- 1) Crop and edge-detect as before ---
        h, w = image.shape[:2]
        cx, cy = w // 2, h // 2
        x1, y1 = max(0, cy - 400), max(0, 445)
        x2, y2 = min(h, cy + 400), min(w, 550)
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
        left_cnt = None
        right_cnt = None

        # --- 3) For each contour, find its closest point to target and bucket by side ---
        for cnt in contours:
            pts = cnt[:, 0]                  # shape=(N,2)
            dists = np.linalg.norm(pts - target, axis=1)
            idx = np.argmin(dists)
            dist = dists[idx]
            x_closest = pts[idx, 0]

            if x_closest < w_e // 2 and dist < left_min:
                left_min, left_cnt = dist, cnt
            elif x_closest > w_e // 2 and dist < right_min:
                right_min, right_cnt = dist, cnt

        # --- 4) Ensure we found both sides ---
        if left_cnt is None or right_cnt is None:
            print("Couldn't find both a left and right contour.")
            return

        # --- 5) Curve-fitting helper ---
        def fit_curve(cnt, deg=3, n_pts=500):
            x = cnt[:, 0, 0]
            y = cnt[:, 0, 1]
            coeffs = np.polyfit(x, y, deg)
            poly   = np.poly1d(coeffs)
            xp     = np.linspace(x.min(), x.max(), n_pts)
            yp     = poly(xp)
            return xp, yp

        # Fit left and right
        xl, yl = fit_curve(left_cnt)
        xr, yr = fit_curve(right_cnt)
        print("Left curve:", len(xl), len(yl))
        print("Right curve:", len(xr), len(yr))

        # --- 6) Draw both in green ---
        curve_img = cv2.cvtColor(edges, cv2.COLOR_GRAY2BGR)
        for xi, yi in zip(xl.astype(int), yl.astype(int)):
            if 0 <= xi < w_e and 0 <= yi < h_e:
                curve_img[yi, xi] = (0, 255, 0)
        for xi, yi in zip(xr.astype(int), yr.astype(int)):
            if 0 <= xi < w_e and 0 <= yi < h_e:
                curve_img[yi, xi] = (0, 255, 0)


        # --- 8) Display and (optionally) return the midpoint poly ---
        #cv2.imshow("Edges", edges)
        cv2.line(curve_img, (int(yl[50]), int(xl[50])), (int(yr[50]), int(xr[50])), (0, 0, 255), 2)
        
        cv2.imshow("Curves", curve_img)
        #cv2.waitKey(1)
        #return

        key = cv2.waitKey(0) & 0xFF  # Mask to handle cross-platform issues

        if key == ord('s'):
            # Save the image
            save_path = './manually_saved_images'
            os.makedirs(save_path, exist_ok=True)
            num_items = len(os.listdir(save_path))
            filename = os.path.join(save_path, f"saved_image_{num_items:05d}.png")

            if not cv2.imwrite(filename, curve_img):
                self.get_logger().error(f"Failed to save image: {filename}")
            else:
                self.get_logger().info(f"Saved {filename}")
                
        else:
            print("Image not saved.")

        # Close the window
        #cv2.destroyAllWindows()

        return 

if __name__ == '__main__':
    rclpy.init()
    test_node = Test()
    rclpy.spin(test_node)
    test_node.destroy_node()
    rclpy.shutdown()