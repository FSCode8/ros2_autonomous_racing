#!/usr/bin/env python3

from image_controller.camera_software import CameraGeometry, CameraGeometry2

import rclpy
from rclpy.node import Node
from rclpy.task import Future

from sensor_msgs.msg import Image
from std_msgs.msg import Float32

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
        
        self.future = None

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
            Float32,
            '/colision_distance',
            10) 
        """

    def test(self):
        # Test the homography matrix
        image = cv.imread("/root/ros2_autonomous_racing/src/image_controller/camera_view.png")
        if image is None:
            self.get_logger().error("Failed to load image")
            return
        print(self.camera_obj.rotation_world_to_cam @ np.array([0, 0, 1]) + np.array([0, 0, 1]))
        print(self.camera_obj.camframe_to_worldframe(np.array([0, 1, 0])))
        print(self.camera_obj.world_normal_camframe)

        # Apply the homography transform
        transformed_image = cv.warpPerspective(image, self.H, (image.shape[1], image.shape[0]))

        # Display the original and transformed images
        cv.imshow("Original Image", image)
        cv.imshow("Transformed Image", transformed_image)
        cv.waitKey(0)
        cv.destroyAllWindows()

    def test_matrix(self):
        # Beispiel-Einheitsvektoren des Quell-Koordinatensystems (A)
        a_x = np.array([1, 0, 0])
        a_y = np.array([0, 1, 0])
        a_z = np.array([0, 0, 1])
        A = np.column_stack((a_x, a_y, a_z))

        # Beispiel-Einheitsvektoren des Ziel-Koordinatensystems (B)
        # (Hier als Beispiel eine 45°-Drehung um Z)
        angle_deg = 45
        angle_rad = np.radians(angle_deg)
        cos_a = np.cos(angle_rad)
        sin_a = np.sin(angle_rad)

        b_x = np.array([0, -1, 0])
        b_y = np.array([0, 0, -1])
        b_z = np.array([0, -1, 0])
        B = np.column_stack((b_x, b_y, b_z))

        # Rotationsmatrix R berechnen
        R = B @ A.T

        # Matrixelemente extrahieren
        r11, r12, r13 = R[0, :]
        r21, r22, r23 = R[1, :]
        r31, r32, r33 = R[2, :]

        # Yaw-Pitch-Roll in ZYX-Reihenfolge
        pitch = np.arctan2(-r31, np.sqrt(r11**2 + r21**2))
        yaw   = np.arctan2(r21, r11)
        roll  = np.arctan2(r32, r33)

        # Ausgabe in Grad
        pitch_deg = np.degrees(pitch)
        yaw_deg   = np.degrees(yaw)
        roll_deg  = np.degrees(roll)

        print(R)
        print(f"Yaw (Z):   {yaw_deg:.2f}°")
        print(f"Pitch (Y): {pitch_deg:.2f}°")
        print(f"Roll (X):  {roll_deg:.2f}°")

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

    #image_transformer.test_matrix()
    #print(image_transformer.camera_obj.camframe_to_worldframe(np.array([1, 0, 0])))
    
    #print(image_transformer.camera_obj.rotation_cam_to_world @ np.array([-1, 0, 0]))
    vec_camframe = np.array([0, 1, 0])
    print("vec_camframe:")
    print(vec_camframe)
    translation_cam_to_world = np.array([0, 0, 1]) # Ursprung der Kamera liegt im Weltkoordinatensystem bei [0, 0, 1]
    print("vec_worldframe:")
    vec_worldframe = image_transformer.camera_obj.rotation_cam_to_world @ vec_camframe + translation_cam_to_world
    print(vec_worldframe)
    """
    c_o = CameraGeometry2()
    print(c_o.rotation_road_to_cam.T @ (vec_worldframe - translation_cam_to_world))
    """
    vec_camframe = image_transformer.camera_obj.rotation_world_to_cam @ (vec_worldframe - translation_cam_to_world)
    print("vec_camframe:")
    print(vec_camframe)
    
    
    #image_transformer.test()

    #rclpy.spin(image_transformer)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    image_transformer.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()