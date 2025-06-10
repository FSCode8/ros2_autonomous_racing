import cv2 
import os
import rclpy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from rclpy.node import Node
import numpy as np
from message_filters import ApproximateTimeSynchronizer, Subscriber

class BirdseyeViewNode(Node):
    def __init__(self):
        super().__init__('birdseye_view_node')
        self.window_name = 'Images'
        self.bridge = CvBridge()
        self.num = 0  # Initialize image counter
        self.image = None

        self.image_dir = '/home/vehicleint/ave_group_1/images' #directionary of images saved
        if not os.path.exists(self.image_dir):
            os.makedirs(self.image_dir)
        
        # Subscriber for the front camera image topic
        self.subscription = self.create_subscription(
            Image,
            '/my_camera/pylon_ros2_camera_node/image_raw_groupb',
            self.image_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.bridge = CvBridge()
        
    def image_callback(self, msg):
        # Convert ROS Image message to OpenCV image
        cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        
        # Define the source and destination points for perspective transform
        # Adjust these points according to your camera setup
        src_points = np.float32([[250, 960], #bottom left
                                 [1030, 960], #bottom right
                                 [750, 450], #top right
                                 [530, 450]]) #top left
        
        dst_points = np.float32([[300, 960], #bottom left
                                 [980, 960], #bottom right
                                 [980, 0], #top right
                                 [300, 0]]) #top left
        
        # Compute the perspective transform matrix
        M = cv2.getPerspectiveTransform(src_points, dst_points)
        
        # Apply the perspective transformation to get bird's eye view
        bev_image = cv2.warpPerspective(cv_image, M, (1280, 960))
        
        # Display the original and transformed images
        cv_image_resized = cv2.resize(cv_image, (540, 720))
        bev_image_resized = cv2.resize(bev_image, (540, 720))
        
        combined_image = np.hstack((cv_image_resized, bev_image_resized))
        cv2.imshow('Birdseye View', combined_image)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = BirdseyeViewNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()