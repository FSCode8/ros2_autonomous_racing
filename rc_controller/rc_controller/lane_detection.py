import cv2 as cv
import rclpy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from rclpy.node import Node
import numpy as np
from std_msgs.msg import Float64

class LaneDetection(Node):
    def __init__(self):
        super().__init__('lane_detection')
        self.max_lowThreshold = 40
        self.window_name = 'Lane Detection'
        self.title_trackbar = 'Min Threshold'
        self.ratio = 3
        self.kernel_size = 3
        self.bridge = CvBridge()
        self.msg_float = Float64()
        
        self.sub = self.create_subscription(
            Image, 
            '/my_camera/pylon_ros2_camera_node/image_raw_groupb', 
            self.image_callback, 
            10
        )
        # self.pub = self.create_publisher(
        #     Image, 
        #     '/my_camera/pylon_ros2_camera_node/lane_detection', 
        #     10
        # )

        self.pub_slope = self.create_publisher(
            Float64, 
            '/lane_detection/slope_value', 
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
        self.CannyThreshold(25)  # Initialize Canny edge detection
        cv.waitKey(1)  # Needed to display images in OpenCV window

    def CannyThreshold(self, val):
        self.low_threshold = val
        self.img_blur = cv.GaussianBlur(self.src_gray, (15, 15), 0)
        detected_edges = cv.Canny(self.img_blur, self.low_threshold, self.low_threshold * self.ratio, self.kernel_size)

        # Masking the region of interest
        height, width = detected_edges.shape
        mask = np.zeros_like(detected_edges)
        
        cm_to_pixels = 2666.67
        y_offset = 5 * cm_to_pixels  # 5 cm in pixels
        # New coordinates for the region of interest
        #print(f"height: {height} and width: {width}")
        polygon = np.array([[(0, 770), #left top
                             (1280, 770), #right top
                             (1280, 800), #right bottom
                             (0, 800)]]) #left bottom
        
        # Adjust these coordinates to better fit your region of interest
        # polygon = np.array([[
        #     (width * 0.1, height),
        #     (width * 0.9, height),
        #     (width * 0.6, height * 0.6),
        #     (width * 0.4, height * 0.6),
        # ]], np.int32)
        cv.fillPoly(mask, polygon, 255)
        masked_edges = cv.bitwise_and(detected_edges, mask)

        # Hough Line Transform to detect lines
        lines = cv.HoughLinesP(masked_edges, 1, np.pi / 180, 30, minLineLength=30, maxLineGap=20)

        # Draw lines on the original image
        line_image = np.zeros_like(self.src)
        if lines is not None:
            slopes_list = []
            detected_slope_list = [0, 0]
            for line in lines:
                x1, y1, x2, y2 = line[0]
                # Filter lines based on slope to remove non-lane lines
                slope = (y2 - y1) / (x2 - x1) if x2 != x1 else 0
                if 0.1 < abs(slope) < 2:  # Adjust slope range as needed
                    #cv.line(line_image, (x1, y1), (x2, y2), (0, 0, 255), 2)
                    if detected_slope_list == [0, 0]:                        
                        detected_slope_list[0] = slope #appends first value nonetheless
                    elif slope < 0 and detected_slope_list[0] > 0:
                        if detected_slope_list[1] <= 0:
                            detected_slope_list[1] = slope #append second value if we don't wanna turn
                    elif slope > 0 and detected_slope_list[0] < 0:
                        if detected_slope_list[1] >= 0:
                            detected_slope_list[1] = slope #append second value if we don't wanna turn

                    slopes_list.append(slope)
                    cv.line(detected_edges, (x1, y1), (x2, y2), (255, 0, 0), 2)
            print(detected_slope_list)
            #average_slope = sum(slopes_list) / len(slopes_list)
            steer = 0.0 #path is straight ahead
            if detected_slope_list[1] == 0:
                if detected_slope_list[0] < 0:
                    steer = -1.0 #path turning right
                else:
                    steer = 1.0 #path turning left
            self.msg_float.data = steer
            self.pub_slope.publish(self.msg_float)
        #dst = cv.addWeighted(self.src, 0.8, line_image, 1, 0)

        #cv.imshow(self.window_name, dst)
        
        # Convert the OpenCV image to ROS Image message and publish
        #lane_image_msg = self.bridge.cv2_to_imgmsg(dst, encoding='bgr8')
        #self.pub.publish(lane_image_msg)

        #cv.imshow(self.window_name, line_image)
        #cv.imshow(self.window_name, detected_edges)
        
        lane_image_msg = self.bridge.cv2_to_imgmsg(detected_edges, encoding='8UC1')
        #self.pub.publish(lane_image_msg)

def main(args=None):
    rclpy.init(args=args)
    lane_detection_node = LaneDetection()
    rclpy.spin(lane_detection_node)

    # Destroy the node explicitly (optional - otherwise it will be done automatically when the garbage collector destroys the node object)
    lane_detection_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

"""
Thresholds: Adjust the low_threshold and max_lowThreshold values to refine edge detection.
Hough Transform Parameters: Fine-tune minLineLength and maxLineGap to detect more relevant lines.
slope Filtering: Modify the slope range to better match the expected angles of your lane lines.
"""