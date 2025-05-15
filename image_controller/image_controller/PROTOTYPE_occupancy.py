import numpy as np
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg  import Header
from geometry_msgs.msg import Pose

import rclpy
from rclpy.node import Node

# meters per pixel in your desired occupancy grid
resolution = 0.05  

# bounds of your grid in world coords
x_min, x_max = -35.0, +35.0
y_min, y_max = -35.0, +35.0

# compute grid size
width  = int((x_max - x_min) / resolution)
height = int((y_max - y_min) / resolution)

grid = OccupancyGrid()
grid.header = Header(frame_id="map")
grid.info.resolution = resolution
grid.info.width  = width
grid.info.height = height

# origin of the grid in the world
grid.info.origin = Pose()
grid.info.origin.position.x = 3
grid.info.origin.position.y = 4
grid.info.origin.position.z = 0.0

# start with unknown
data = np.full((height, width), -1, dtype=np.int8)

# for each pixel in your probability map…
for i in range(P.shape[0]):
    for j in range(P.shape[1]):
        prob = P[i, j]

        # compute world coords of this pixel
        x, y, z = pixel_to_world(i, j)   # your reprojection

        # compute grid indices
        gx = int((x - x_min) / resolution)
        gy = int((y - y_min) / resolution)

        # check bounds
        if 0 <= gx < width and 0 <= gy < height:
            # decide occupancy: e.g.
            if prob > 0.5:
                # “lane” pixels as free
                data[gy, gx] = int((1.0 - prob) * 99)  
                # (higher prob → lower cost)
            else:
                # non‑lane as occupied
                data[gy, gx] = 100

class LaneGridPublisher(Node):
    def __init__(self):
        super().__init__('lane_grid_pub')
        self.pub = self.create_publisher(OccupancyGrid, 'lane_grid', 10)
        self.timer = self.create_timer(0.5, self.publish_grid)

    def publish_grid(self):
        # assume grid is already computed (above)
        grid.header.stamp = self.get_clock().now().to_msg()
        self.pub.publish(grid)

def main():
    rclpy.init()
    node = LaneGridPublisher()
    rclpy.spin(node)
    rclpy.shutdown()