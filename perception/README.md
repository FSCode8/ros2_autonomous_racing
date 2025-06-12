# Package Name
perception
A comprehensive ROS2 package for autonomous racing perception, providing computer vision algorithms for lane detection, image processing, and coordinate transformations between camera and world frames.

## Installation

The perception package requires specific ROS2 and Python dependencies for computer vision processing. Install these components before building the package.

### Prerequisites

- ROS2 Humble installed
- Gazebo Fortress

### Build Instructions

1. Navigate to your ROS2 workspace:
     ```bash
     cd ~/ros2_autonomous_racing
     ```

2. Build the package:
     ```bash
     colcon build --packages-select perception
     ```

3. Source the workspace:
     ```bash
     source install/setup.bash
     ```

4. Install dependencies:
     ```bash
     rosdep install --from-paths src -y --ignore-src
     ```

## Package Contents

### Core Modules

#### `run_perception.py`
Main executable script for the perception pipeline. Handles ROS2 node initialization, of the image_processing and the OccupancyGrid.

#### `image_processing.py`
Core image processing module using Lane detection with computer vision routines or ith the classical approach. 

#### `prototype.py`
Development and testing node with comprehensive lane detection pipeline. Includes edge detection, contour analysis, and action client integration for path following.
Implements the full setup routine (taken out in image_processing.py)

#### `LaneDetection.py`
Main lane detection 
Module using PyTorch neural networks for semantic segmentation. Provides polynomial fitting for lane boundaries and path generation for autonomous navigation.
Also implementation of lane detection in the classical way.

#### `VisionCalculation.py`
Computer vision utility classes for coordinate transformations between camera and world frames. Includes homography calculations, and bird's-eye view projections.

#### `OccupancyGridNode.py`
ROS2 node for publishing occupancy grid maps based on lane detection results. Manages grid updates.


### Supporting Files

#### `run_trough_script.py`
Allows to set the position of the vehicle in the simulation.

