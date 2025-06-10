import cv2 as cv
import os
import rclpy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from rclpy.node import Node

class GetImages(Node):
    def __init__(self):
        super().__init__('get_images')
        self.window_name = 'Images'
        self.bridge = CvBridge()
        self.num = 0  # Initialize image counter
        self.image = None

        self.image_dir = '/home/vehicleint/ave_group_1/images' #directionary of images saved
        if not os.path.exists(self.image_dir):
            os.makedirs(self.image_dir)
        
        self.sub = self.create_subscription(
            Image, 
            '/my_camera/pylon_ros2_camera_node/image_raw_groupb', 
            self.image_callback, 
            10
        )

    def image_callback(self, msg):
        self.image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        if self.image is None:
            self.get_logger().error('Could not open or find the image.')
            return
        
        cv.imshow(self.window_name, self.image)
        k = cv.waitKey(1)
        
        if k == 27:  # 'ESC' key to exit
            cv.destroyAllWindows()
            rclpy.shutdown()
        elif k == ord('s'):  # 's' key to save the image
            self.image = cv.cvtColor(self.image, cv.COLOR_BGR2GRAY)
            cv.imwrite(f'/home/vehicleint/ave_group_1/images/img{self.num}.png', self.image)
            print(f"Image saved as /home/vehicleint/ave_group_1/images/img{self.num}.png")
            self.num += 1

def main(args=None):
    rclpy.init(args=args)
    get_images_node = GetImages()
    try:
        rclpy.spin(get_images_node)
    except KeyboardInterrupt:
        pass
    finally:
        get_images_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
