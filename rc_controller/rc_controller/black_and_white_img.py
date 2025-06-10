import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class LaneDetectionNode(Node):
    def __init__(self):
        super().__init__('lane_detection_node')
        
        self.sub = self.create_subscription(
            Image,
            '/my_camera/pylon_ros2_camera_node/image_raw_groupb',
            self.image_callback,
            10
        )
        self.pub = self.create_publisher(
            Image,
            '/my_camera/pylon_ros2_camera_node/lane_detection',
            10
        )
        self.bridge = CvBridge()
        self.window_name = 'Lane Detection'
        
        # Define the polygon for the area of interest
        self.polygon = np.array([[(0, 550), (1280, 550), (1280, 800), (0, 800)]])

        # Create OpenCV window
        cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(self.window_name, 600, 400)

    def apply_roi(self, image):
        # Create a mask for the ROI
        mask = np.zeros_like(image)
        cv2.fillPoly(mask, [self.polygon], 255)
        # Apply the mask to the image
        masked_image = cv2.bitwise_and(image, mask)
        return masked_image

    def image_callback(self, msg):
        # Convert ROS Image message to OpenCV image
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # Convert image to grayscale
        gray_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

        # Apply the threshold
        _, bw_image = cv2.threshold(gray_image, 100, 255, cv2.THRESH_BINARY)

        # Apply ROI to the binary image
        bw_image_roi = self.apply_roi(bw_image)

        # Only keep the white regions
        result_image = cv2.bitwise_and(cv_image, cv_image, mask=bw_image_roi)

        # Display the processed image
        cv2.imshow(self.window_name, result_image)
        cv2.waitKey(1)

        # Convert OpenCV image back to ROS Image message
        result_msg = self.bridge.cv2_to_imgmsg(result_image, encoding='8UC3')

        # Publish the processed image
        self.pub.publish(result_msg)

def main(args=None):
    rclpy.init(args=args)
    node = LaneDetectionNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
