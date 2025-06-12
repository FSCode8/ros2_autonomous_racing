#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rclpy.executors import ExternalShutdownException

from perception.VisionCalculation import Camera
from perception.VisionCalculation import VehicleGeometry
from perception.VisionCalculation import VisionCalculation

from perception.OccupancyGridNode import LaneGridPublisher
from perception.image_processing import ImageProcessor

if __name__ == '__main__':
    rclpy.init(args=None)
    
    # Run
    try:
        image_processor_node = ImageProcessor()
        occupancy_grid_node = LaneGridPublisher()
        
        # Create single-threaded executor and add both nodes
        executor = SingleThreadedExecutor()
        executor.add_node(image_processor_node)
        executor.add_node(occupancy_grid_node)
        
        # Spin both nodes on single thread
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass  # Handle Ctrl+C gracefully
    finally:
        # Clean up
        if 'image_processor_node' in locals():
            image_processor_node.destroy_node()

        if 'occupancy_grid_node' in locals():
            occupancy_grid_node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()