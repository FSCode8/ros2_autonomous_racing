#!/usr/bin/env python3

from image_controller.camera_software import CameraGeometry

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
    Cv = 400.0
    return np.array([[alpha, 0, Cu],
                     [0, alpha, Cv],
                     [0, 0, 1.0]])

class ImageTransformer(Node):

    def __init__(self):
        super().__init__('image_transformer')
        # NOTE: Add server for camera_info?
        
        self.future = None

        self.bridge = CvBridge()

        self.camera_obj = CameraGeometry(K_matrix=get_intrinsic_matrix())

        self.min_carless_pixel = int(self.camera_obj.get_min_carless_pixel(shadow_point = np.array([0, 1, 3.22]))[1])

        # Create homography matrix
        """
        K = self.camera_obj.intrinsic_matrix
        R_t1 = self.camera_obj.trafo_perspective_cam
        R_t2 = self.camera_obj.trafo_perspective_bird
        self.H = self.camera_obj.create_perspective_homography(K1=K, K2=K, R_t1=R_t1, R_t2=R_t2)"""
        
        
        self.subscription = self.create_subscription(
            Image,
            '/image_raw',
            self.execute_callback,
            10)
       
        # Publisher for transformed image
        self.publisher_ = self.create_publisher(
            Float32,
            '/collision_distance',
            10) 
        self.get_logger().info("ImageTransformer node has been started.")

    def test(self):
 
        """#print(self.camera_obj.uv_to_XYZ_camframe(0, 0))
        print(self.camera_obj.uv_to_XYZ_camframe(400, 800))
        print(self.camera_obj.uv_to_XYZ_camframe(400, 700))
        print(self.camera_obj.uv_to_XYZ_camframe(400, 600))
        print(self.camera_obj.uv_to_XYZ_camframe(400, 547))
        print(self.camera_obj.uv_to_XYZ_camframe(400, 500))
        print(self.camera_obj.uv_to_XYZ_camframe(400, 401))"""
        

        print(self.min_carless_pixel)

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
                min_distance = self.camera_obj.compute_dist(400, v) # (u, v)
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