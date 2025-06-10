import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from cv_bridge import CvBridge
from pypylon import pylon
import cv2
import numpy as np

class ColorDetectionNode(Node):
    def __init__(self):
        super().__init__('color_detection')
        self.bridge = CvBridge()
        self.pub = self.create_publisher(String, '/color_detection', 10)

        # Basler camera setup
        self.camera = pylon.InstantCamera(pylon.TlFactory.GetInstance().CreateFirstDevice())
        self.camera.StartGrabbing(pylon.GrabStrategy_LatestImageOnly)
        self.converter = pylon.ImageFormatConverter()
        self.converter.OutputPixelFormat = pylon.PixelType_BGR8packed
        self.converter.OutputBitAlignment = pylon.OutputBitAlignment_MsbAligned

        # Camera calibration parameters (example values, replace with your own)
        #self.camera_matrix = np.array([[92.64607068, 0, 765.89916204], 
        #                              [0, 151.34161265, 665.90382887], 
        #                              [0, 0, 1]], dtype=np.float32)
        #self.dist_coeffs = np.array([602, 452, 331, 407], dtype=np.float32)

        # Define the size of the central region (width, height)
        self.region_width = 300
        self.region_height = 300

        self.timer = self.create_timer(0.1, self.image_callback)  # Adjust the rate as needed

    def image_callback(self):
        if self.camera.IsGrabbing():
            grabResult = self.camera.RetrieveResult(5000, pylon.TimeoutHandling_ThrowException)
            if grabResult.GrabSucceeded():
                # Access the image data
                image = self.converter.Convert(grabResult)
                img = image.GetArray()

                # Check if the image is valid
                if img is None or img.size == 0:
                    self.get_logger().warn("Received an empty image.")
                    grabResult.Release()
                    return

                # Display the raw image for debugging
                #cv2.imshow('Raw Image', img)
                #cv2.waitKey(1)  # Refresh the display

                # Undistort the image using camera calibration parameters
                #undistorted_img = cv2.undistort(img, self.camera_matrix, self.dist_coeffs)

                # Check if undistortion worked
                #if undistorted_img is None or undistorted_img.size == 0:
                #    self.get_logger().warn("Undistorted image is empty.")
                #    grabResult.Release()
                #    return

                # Get image dimensions
                height, width, _ = img.shape

                # Define the bounding box for the central region
                x_start = (width - self.region_width) // 2
                y_start = (height - self.region_height) // 2
                x_end = x_start + self.region_width
                y_end = y_start + self.region_height

                # Crop the image to the central region
                cropped_img = img[y_start:y_end, x_start:x_end]

                # Check if cropping worked
                if cropped_img is None or cropped_img.size == 0:
                    self.get_logger().warn("Cropped image is empty.")
                    grabResult.Release()
                    return

                # Preprocess the image
                blurred_img = cv2.GaussianBlur(cropped_img, (5, 5), 0)

                # Detect colors
                detected_color = self.detect_colors(blurred_img)

                # Publish the detected color
                self.pub.publish(String(data=detected_color))

                # Show the cropped image with the detected color
                self.display_color_detection(cropped_img, detected_color)

            grabResult.Release()

    def detect_colors(self, img):
        # Convert the image to HSV color space
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        # Debug: Show HSV image
        cv2.imshow('HSV Image', hsv)
        cv2.waitKey(1)  # Refresh the display

        # Define color ranges for red, yellow, green in HSV
        red_lower1 = np.array([0, 100, 100])
        red_upper1 = np.array([10, 255, 255])
        red_lower2 = np.array([160, 100, 100])
        red_upper2 = np.array([180, 255, 255])

        yellow_lower = np.array([20, 100, 100])
        yellow_upper = np.array([30, 255, 255])

        green_lower = np.array([40, 100, 100])
        green_upper = np.array([80, 255, 255])

        # Create masks for each color
        red_mask1 = cv2.inRange(hsv, red_lower1, red_upper1)
        red_mask2 = cv2.inRange(hsv, red_lower2, red_upper2)
        red_mask = cv2.bitwise_or(red_mask1, red_mask2)

        yellow_mask = cv2.inRange(hsv, yellow_lower, yellow_upper)
        green_mask = cv2.inRange(hsv, green_lower, green_upper)

        # Apply morphological operations to clean up masks
        kernel = np.ones((5, 5), np.uint8)
        red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_CLOSE, kernel)
        yellow_mask = cv2.morphologyEx(yellow_mask, cv2.MORPH_CLOSE, kernel)
        green_mask = cv2.morphologyEx(green_mask, cv2.MORPH_CLOSE, kernel)

        # Debug: Show color masks
        cv2.imshow('Red Mask', red_mask)
        cv2.imshow('Yellow Mask', yellow_mask)
        cv2.imshow('Green Mask', green_mask)
        cv2.waitKey(1)  # Refresh the display

        # Count non-zero pixels for each color
        red_count = cv2.countNonZero(red_mask)
        yellow_count = cv2.countNonZero(yellow_mask)
        green_count = cv2.countNonZero(green_mask)

        # Print counts for debugging
        self.get_logger().info(f"Red count: {red_count}, Yellow count: {yellow_count}, Green count: {green_count}")

        # Determine the detected color
        if red_count > 100:
            return "Red"
        elif yellow_count > 100:
            return "Yellow"
        elif green_count > 100:
            return "Green"
        else:
            return "No Color Detected"

    def display_color_detection(self, img, detected_color):
        # Draw a rectangle around the detected color area
        cv2.rectangle(img, (0, 0), (img.shape[1], img.shape[0]), (255, 255, 255), 2)

        # Display the detected color on the image
        font = cv2.FONT_HERSHEY_SIMPLEX
        font_scale = 1
        font_thickness = 2
        color = (0, 0, 0)  # Black color in BGR

        # Define the text size
        text_size = cv2.getTextSize(detected_color, font, font_scale, font_thickness)[0]
        text_x = (img.shape[1] - text_size[0]) // 2
        text_y = (img.shape[0] + text_size[1]) // 2

        # Put the text on the image
        cv2.putText(img, detected_color, (text_x, text_y), font, font_scale, color, font_thickness, cv2.LINE_AA)

        # Show the image
        cv2.imshow('Color Detection', img)
        cv2.waitKey(1)  # Refresh the display

def main(args=None):
    rclpy.init(args=args)
    node = ColorDetectionNode()
    rclpy.spin(node)

    # Clean up
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
