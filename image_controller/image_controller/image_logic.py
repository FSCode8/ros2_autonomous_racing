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

import cv2 as cv
import numpy as np
from cv_bridge import CvBridge, CvBridgeError


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

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.bridge = CvBridge()

        # Create subscription with the callback group
        self.subscription = self.create_subscription(
            CameraInfo,  # Replace with your message type
            '/camera_info',
            self._initial_data_callback,
            10
        )

        # Publisher for transformed image
        self.publisher_ = self.create_publisher(
            Float32,
            '/collision_distance',
            10) 
        self.get_logger().info("ImageTransformer node has been started.")

        # Wait for the initial data
        self.get_logger().info('Waiting for initial data from topic...')

    def _initial_data_callback(self, msg):
        
        self._intrinsic_matrix = array_3x3 = np.array(msg.k).reshape(3, 3)
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
        
        # Assume your np.array point
        np_point = np.array([0.0, 0.0, 1.0])

        # Convert the quaternion to a rotation matrix
        print(f"Quaternion: {rot.x}, {rot.y}, {rot.z}, {rot.w}")
        print(f"Translation: {trans.x}, {trans.y}, {trans.z}")
        rotation_matrix = VisionCalculation.quaternion_to_rotation_matrix(rot.x, rot.y, rot.z, rot.w)
        
        # Transform the point
        print(f"Rotation matrix: {rotation_matrix}")
        transformed_np_point = rotation_matrix @ np_point + np.array([trans.x, trans.y, trans.z])

        self.get_logger().info(f"Transformed point: {transformed_np_point}")
        
        translation_vector = np.array([trans.x-1, trans.y, trans.z])    # world frame origin is under camera

        self.vision_calc = VisionCalculation(
            camera_object=camera_obj,
            vehicle_object=vehicle_obj,
            rotation_cam_to_world=rotation_matrix,
            translation_cam_to_world=translation_vector
        )

        self.min_carless_pixel = int(self.vision_calc.get_min_carless_pixel()[1]) 

        self.subscription = self.create_subscription(
            Image,
            '/image_raw',
            self.execute_callback,
            10
        )
    

    def test(self, image):

        from_frame_rel = 'base_link'
        to_frame_rel = 'camera_link_optical'
        print(from_frame_rel)
        # First, check what frames are available
        frames = self.tf_buffer.all_frames_as_string()
        self.get_logger().info(f'Available frames: {frames}')

        try:
            t = self.tf_buffer.lookup_transform(
                from_frame_rel,
                to_frame_rel,
                rclpy.time.Time())
        except TransformException as ex:
            self.get_logger().warn(
                f'Could not transform {from_frame_rel} to {to_frame_rel}: {ex}')
            return     

        print(f"Transform from {to_frame_rel} to {from_frame_rel}:")
        print(t.transform.translation)
        print(f"Rotation: {t.transform.rotation.x}, {t.transform.rotation.y}, {t.transform.rotation.z}, {t.transform.rotation.w}")
        return
        
        # Test the homography matrix
        image = cv.imread("/root/ros2_autonomous_racing/src/image_controller/camera_view.png")
        
        print(self.compute_min_distance(image))
        
        return
        """print(self.camera_obj.rotation_world_to_cam @ np.array([0, 0, 1]) + np.array([0, 0, 1]))
        print(self.camera_obj.camframe_to_worldframe(np.array([0, 1, 0])))
        print(self.camera_obj.world_normal_camframe)"""

        # Apply the homography transform
        #transformed_image = cv.warpPerspective(image, self.H, (image.shape[1], image.shape[0]))
        transformed_image = image.copy()
        transformed_image[200:205, 100:105] = [255, 0, 0]  # Draw a red square on the transformed image
        # Display the original and transformed images
        cv.imshow("Original Image", image)
        cv.imshow("Transformed Image", transformed_image)
        cv.waitKey(0)
        cv.destroyAllWindows()

    def compute_min_distance(self, image):
        # Convert the image to grayscale
        gray = cv.cvtColor(image, cv.COLOR_BGR2GRAY)

        # Threshold the image to create a binary image
        _, binary = cv.threshold(gray, 190, 255, cv.THRESH_BINARY)

        #binary[547, 0:800] = 255

        # Initialize minimum distance
        min_distance = float(-1)

        # Loop through each contour
        for v in range(self.min_carless_pixel, 400, -1):  # from 800 to 0 inclusive
            if binary[v, 400] == 255:    # (v, u) == (y, x)
                # Calculate the distance from the contour to the camera
                min_distance = self.vision_calc.compute_dist(400, v) # (u, v)
                print(v)
                return min_distance   

        return min_distance     

    def execute_callback(self, msg):
        self.get_logger().info('Receiving video frame')
        # Convert ROS Image message to OpenCV2 image
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except CvBridgeError as e:
            self.get_logger().error(f'CvBridge Error: {e}')
        
        #transformed_image = cv.warpPerspective(image, H, (image.shape[1], image.shape[0]))
        #cv.imwrite("/root/ros2_autonomous_racing/camera_view.png", cv_image)

        # compute minimum distance
        compute_min_distance = self.compute_min_distance(cv_image)
        self.get_logger().info(f'Minimum distance: {compute_min_distance}')
        msg = Float32()
        msg.data = compute_min_distance
        self.publisher_.publish(msg)
        # Display image
        cv.imshow("Camera Feed", cv_image)
        cv.waitKey(1)


def main(args=None):
    rclpy.init(args=args)

    image_transformer = ImageTransformer()
    #image_transformer.test()
    #image_transformer.camera_obj.test_camera_geometry(
    #    test_vec_camframe=np.array([1, 0, 0]),
    #    test_vec_worldframe=np.array([0, 1, 0])
    #)

    #print(image_transformer.camera_obj.uv_to_XYZ_worldframe(400, 800))

    #image_transformer.test()

    rclpy.spin(image_transformer)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    image_transformer.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()