from nav_msgs.msg import OccupancyGrid
#from std_msgs.msg  import Header
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from cv_bridge import CvBridge

import rclpy
from rclpy.node import Node

import numpy as np



class LaneGridPublisher(Node):
    def __init__(self):
        super().__init__('lane_grid_pub')

        ### Create occupancy grid ###
        resolution = 0.1
        x_min, x_max = -10.0, +10.0
        y_min, y_max = -10.0, +10.0
        width  = int((x_max - x_min) / resolution)
        height = int((y_max - y_min) / resolution)

        self.grid = OccupancyGrid()
        #self.grid.header = Header()
        self.grid.header.frame_id = "odom"
        self.grid.info.map_load_time = self.get_clock().now().to_msg()
        self.grid.info.resolution = resolution
        self.grid.info.width  = width
        self.grid.info.height = height
        self.grid.info.origin.position.x = 0.0
        self.grid.info.origin.position.y = 0.0
        self.grid.info.origin.position.z = 0.0

        data = np.full((height, width), 0, dtype=np.int8) # All allowed
        self.grid.data = data.ravel().tolist()
        
        ### Create initial position ###
        self.vehicle_pose = PoseStamped()

        self.vehicle_pose.header.frame_id = "base_link"
        self.vehicle_pose.header.stamp = self.get_clock().now().to_msg()

         # Set position (x, y, z in meters)
        self.vehicle_pose.pose.position.x = 0.0
        self.vehicle_pose.pose.position.y = 0.0
        self.vehicle_pose.pose.position.z = 0.0
        
        # Set orientation (quaternion: x, y, z, w) to no rotation
        self.vehicle_pose.pose.orientation.x = 0.0
        self.vehicle_pose.pose.orientation.y = 0.0
        self.vehicle_pose.pose.orientation.z = 0.0
        self.vehicle_pose.pose.orientation.w = 1.0
        
        ### Create cv bridge ###
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.bridge = CvBridge()

        ### Create publisher and subscriber ###
        # create publisher for the occupancy grid
        self.pub = self.create_publisher(OccupancyGrid, '/lane_grid', 10)
        self.timer = self.create_timer(0.5, self.publish_grid)

        # create subscriber to the vehicle_position
        self.pose_sub = self.create_subscription(PoseStamped, '/vehicle_pose', self.vehicle_pose_update, 10)

        # create subscriber to the
        self.perception_update = self.create_subscription(Image, '/occ_grid_update', self.grid_update, 10)

    def grid_update(self, msg):
        """
        Update the grid with received perception data and current position.
        """
        pass

    def publish_grid(self):
        # assume grid is already computed (above)
        self.grid.header.stamp = self.get_clock().now().to_msg()
        self.pub.publish(self.grid)
        self.get_logger().info("Grid published on /lane_grid")

    def vehicle_pose_update(self, msg):
        self.vehicle_pose = msg
