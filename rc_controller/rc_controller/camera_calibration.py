import cv2 as cv
import rclpy
from sensor_msgs.msg import Image
from rclpy.node import Node
import numpy as np

class Calibration(Node):
    def __init__(self):
        super().__init__('calibration')
        self.mtx = np.array([[587.02559142, 0., 632.03759266], [0., 587.22159019, 512.96361401], [0., 0., 1.]])
        self.dist = np.array([[-3.29641569e-01,  1.37815847e-01,  4.94936421e-04, -6.79209203e-05, -3.06499684e-02]])
        
        self.sub = self.create_subscription(
            Image, 
            '/my_camera/pylon_ros2_camera_node/image_raw_groupb', 
            self.image_callback, 
            10
        )
        self.pub = self.create_publisher(
            Image, 
            '/my_camera/pylon_ros2_camera_node/calibrated_image_raw_groupb', 
            10
        )

    def image_callback(self, msg):

        # Convert ROS Image message to OpenCV image
        img = np.array(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, -1)
        
        h,  w = img.shape[:2]
        newcameramtx, roi = cv.getOptimalNewCameraMatrix(self.mtx, self.dist, (w,h), 1, (w,h))
        dst = cv.undistort(img, self.mtx, self.dist, None, newcameramtx)

        # crop the image
        x, y, w, h = roi
        dst = dst[y:y+h, x:x+w]

        # Convert back to ROS Image message (this can probably be adjusted to your specific use case)
        output_msg = Image()
        output_msg.height = dst.shape[0]
        output_msg.width = dst.shape[1]
        output_msg.encoding = "rgb8"
        output_msg.data = dst.flatten().tolist()
        self.pub.publish(output_msg)
   

def main(args=None):
    rclpy.init(args=args)
    calibration = Calibration()
    rclpy.spin(calibration)

    # Destroy the node explicitly (optional - otherwise it will be done automatically when the garbage collector destroys the node object)
    calibration.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
