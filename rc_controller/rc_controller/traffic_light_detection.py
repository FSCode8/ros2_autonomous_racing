import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
from pypylon import pylon
import cv2
import numpy as np

class TrafficLightDetectionNode(Node):
    def __init__(self):
        super().__init__('traffic_light_detection')
        self.bridge = CvBridge()
        self.pub = self.create_publisher(String, '/traffic_light_state', 10)

        # Basler camera setup
        self.camera = pylon.InstantCamera(pylon.TlFactory.GetInstance().CreateFirstDevice())
        self.camera.StartGrabbing(pylon.GrabStrategy_LatestImageOnly)
        self.converter = pylon.ImageFormatConverter()
        self.converter.OutputPixelFormat = pylon.PixelType_BGR8packed
        self.converter.OutputBitAlignment = pylon.OutputBitAlignment_MsbAligned

        self.timer = self.create_timer(0.1, self.image_callback)  # Adjust the rate as needed

        # Define the bounding box (x, y, width, height)
        self.bounding_box = (100, 100, 300, 300)  # Adjust these values as needed

    def image_callback(self):
        if self.camera.IsGrabbing():
            grabResult = self.camera.RetrieveResult(5000, pylon.TimeoutHandling_ThrowException)
            if grabResult.GrabSucceeded():
                # Access the image data
                image = self.converter.Convert(grabResult)
                img = image.GetArray()

                # Crop the image to the bounding box
                x, y, w, h = self.bounding_box
                cropped_img = img[y:y+h, x:x+w]

                # Detect traffic light
                state = self.detect_traffic_light(cropped_img)

                # Publish the traffic light state
                self.pub.publish(String(data=state))

                # Show the image with detection results
                self.display_traffic_light_detection(cropped_img, state)

            grabResult.Release()

    def detect_traffic_light(self, img):
        # Convert the image to HSV color space
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        # Define color ranges for red, yellow, green in HSV
        red_lower1 = np.array([0, 120, 70])
        red_upper1 = np.array([10, 255, 255])
        red_lower2 = np.array([170, 120, 70])
        red_upper2 = np.array([180, 255, 255])

        yellow_lower = np.array([12, 55, 80])
        yellow_upper = np.array([32, 255, 255])

        green_lower = np.array([50, 5, 150])
        green_upper = np.array([86, 250, 250])

        # Create masks for each color
        red_mask1 = cv2.inRange(hsv, red_lower1, red_upper1)
        red_mask2 = cv2.inRange(hsv, red_lower2, red_upper2)
        red_mask = cv2.bitwise_or(red_mask1, red_mask2)

        yellow_mask = cv2.inRange(hsv, yellow_lower, yellow_upper)
        green_mask = cv2.inRange(hsv, green_lower, green_upper)

        # Check for the presence of each color
        if cv2.countNonZero(red_mask) > 50:
            return "Red"
        elif cv2.countNonZero(yellow_mask) > 50:
            return "Yellow"
        elif cv2.countNonZero(green_mask) > 50:
            return "Green"
        else:
            return "No Traffic Light Detected"

    def display_traffic_light_detection(self, img, state):
        # Display the detected traffic light state on the image
        font = cv2.FONT_HERSHEY_SIMPLEX
        font_scale = 1
        font_thickness = 3  # Increase thickness for boldness
        color = (0, 0, 0)  # Black color in BGR

        # Get the text size to center it if needed
        text_size = cv2.getTextSize(state, font, font_scale, font_thickness)[0]
        text_x = 50
        text_y = 50 + text_size[1]  # Adjust as needed

        # Put the text on the image
        cv2.putText(img, state, (text_x, text_y), font, font_scale, color, font_thickness, cv2.LINE_AA)

        # Show the image
        cv2.imshow('Traffic Light Detection', img)
        cv2.waitKey(1)  # Refresh the display

def main(args=None):
    rclpy.init(args=args)
    node = TrafficLightDetectionNode()
    rclpy.spin(node)

    # Clean up
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
