import cv2 as cv
import rclpy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from rclpy.node import Node

class EdgeDetection(Node):
    def __init__(self):
        super().__init__('edge_detection')
        self.max_lowThreshold = 100
        self.window_name = 'Edge Map'
        self.title_trackbar = 'Min Threshold'
        self.ratio = 3
        self.kernel_size = 3
        self.bridge = CvBridge()
        
        self.sub = self.create_subscription(
            Image, 
            '/my_camera/pylon_ros2_camera_node/image_raw_groupb', 
            self.image_callback, 
            10
        )
        self.pub = self.create_publisher(
            Image, 
            '/my_camera/pylon_ros2_camera_node/edge_detection', 
            10
        )
        
        cv.namedWindow(self.window_name)
        cv.createTrackbar(self.title_trackbar, self.window_name, 0, self.max_lowThreshold, self.CannyThreshold)

    def image_callback(self, msg):
        self.src = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        if self.src is None:
            self.get_logger().error('Could not open or find the image.')
            return

        self.src_gray = cv.cvtColor(self.src, cv.COLOR_BGR2GRAY)
        self.CannyThreshold(20)  # Initialize Canny edge detection
        cv.waitKey(1)  # Needed to display images in OpenCV window

    def CannyThreshold(self, val):
        self.low_threshold = val
        self.img_blur = cv.blur(self.src_gray, (3, 3))
        detected_edges = cv.Canny(self.img_blur, self.low_threshold, self.low_threshold * self.ratio, self.kernel_size)
        mask = detected_edges != 0
        dst = self.src * (mask[:, :, None].astype(self.src.dtype))

        cv.imshow(self.window_name, dst)
        
        # Convert the OpenCV image to ROS Image message and publish
        edge_image_msg = self.bridge.cv2_to_imgmsg(dst, encoding='bgr8')
        self.pub.publish(edge_image_msg)

def main(args=None):
    rclpy.init(args=args)
    edge_detection_node = EdgeDetection()
    rclpy.spin(edge_detection_node)

    # Destroy the node explicitly (optional - otherwise it will be done automatically when the garbage collector destroys the node object)
    edge_detection_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
