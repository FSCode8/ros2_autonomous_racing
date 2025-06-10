import cv2 as cv
import rclpy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from rclpy.node import Node
import numpy as np

class LaneDetection2(Node):
    def __init__(self):
        super().__init__('lane_detection')
        self.max_lowThreshold = 20
        self.window_name = 'Lane Detection'
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
            '/my_camera/pylon_ros2_camera_node/lane_detection', 
            10
        )
        
        cv.namedWindow(self.window_name)
        cv.createTrackbar(self.title_trackbar, self.window_name, 0, self.max_lowThreshold, self.CannyThreshold)
        self.prev_left_line = None
        self.prev_right_line = None

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
        self.img_blur = cv.GaussianBlur(self.src_gray, (5, 5), 0)
        mean_val = np.mean(self.img_blur)
        self.low_threshold = int(0.66 * mean_val)
        detected_edges = cv.Canny(self.img_blur, self.low_threshold, self.low_threshold * self.ratio, self.kernel_size)

        # Masking the region of interest
        height, width = detected_edges.shape
        mask = np.zeros_like(detected_edges)
        polygon = np.array([[(300, 550), 
                            (980, 550), 
                            (1280, 800), 
                            (0, 800)]])
        cv.fillPoly(mask, polygon, 255)
        masked_edges = cv.bitwise_and(detected_edges, mask)

        # Hough Line Transform to detect lines
        lines = cv.HoughLinesP(masked_edges, 1, np.pi / 180, 30, minLineLength=10, maxLineGap=200)

        # Draw lines on the original image
        line_image = np.zeros_like(self.src)
        left_lines = []
        right_lines = []

        if lines is not None:
            for line in lines:
                x1, y1, x2, y2 = line[0]
                slope = (y2 - y1) / (x2 - x1) if x2 != x1 else 0
                intercept = y1 - slope * x1
                if 0.1 < abs(slope) < 2:
                    if slope < 0:
                        left_lines.append((slope, intercept))
                    else:
                        right_lines.append((slope, intercept))

        if left_lines:
            left_slope, left_intercept = np.mean(left_lines, axis=0)
            left_line = (left_slope, left_intercept)
            left_line = self.smooth_lines([left_line], [self.prev_left_line])[0]
            self.prev_left_line = left_line
            left_slope, left_intercept = left_line
            left_y1 = height
            left_x1 = int((left_y1 - left_intercept) / left_slope)
            left_y2 = int(height * 0.6)
            left_x2 = int((left_y2 - left_intercept) / left_slope)
            cv.line(line_image, (left_x1, left_y1), (left_x2, left_y2), (255, 0, 0), 10)

        if right_lines:
            right_slope, right_intercept = np.mean(right_lines, axis=0)
            right_line = (right_slope, right_intercept)
            right_line = self.smooth_lines([right_line], [self.prev_right_line])[0]
            self.prev_right_line = right_line
            right_slope, right_intercept = right_line
            right_y1 = height
            right_x1 = int((right_y1 - right_intercept) / right_slope)
            right_y2 = int(height * 0.6)
            right_x2 = int((right_y2 - right_intercept) / right_slope)
            cv.line(line_image, (right_x1, right_y1), (right_x2, right_y2), (255, 0, 0), 10)

        # Polynomial fitting for curved lines
        points = []
        if lines is not None:
            for line in lines:
                x1, y1, x2, y2 = line[0]
                points.append([x1, y1])
                points.append([x2, y2])
            points = np.array(points)
            x = points[:, 0]
            y = points[:, 1]
            if len(x) > 0 and len(y) > 0:
                curve_fit = np.polyfit(y, x, 2)
                fit_fn = np.poly1d(curve_fit)
                y_new = np.linspace(min(y), max(y), num=500)
                x_new = fit_fn(y_new).astype(int)
                for i in range(len(y_new) - 1):
                    cv.line(line_image, (x_new[i], int(y_new[i])), (x_new[i + 1], int(y_new[i + 1])), (0, 255, 0), 2)

        dst = cv.addWeighted(self.src, 0.8, line_image, 1, 0)

        # Convert the OpenCV image to ROS Image message and publish
        lane_image_msg = self.bridge.cv2_to_imgmsg(dst, encoding='bgr8')
        self.pub.publish(lane_image_msg)

        cv.imshow(self.window_name, dst)

    def smooth_lines(self, lines, prev_lines, alpha=0.2):
        if prev_lines is None or len(prev_lines) == 0:
            return lines
        smoothed_lines = []
        for new, old in zip(lines, prev_lines):
            if old is not None:
                smoothed_line = (1 - alpha) * np.array(old) + alpha * np.array(new)
            else:
                smoothed_line = new
            smoothed_lines.append(tuple(smoothed_line))
        return smoothed_lines

def main(args=None):
    rclpy.init(args=args)
    lane_detection_node = LaneDetection2()
    rclpy.spin(lane_detection_node)

    # Destroy the node explicitly (optional - otherwise it will be done automatically when the garbage collector destroys the node object)
    lane_detection_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
