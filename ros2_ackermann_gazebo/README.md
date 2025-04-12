Repo is directly based on https://github.com/ros-controls/gz_ros2_control/tree/humble/gz_ros2_control_demos and https://control.ros.org/humble/doc/gz_ros2_control/doc/index.html

# Setup
1. Install Gazebo Fortress
See: https://gazebosim.org/docs/fortress/install_ubuntu/
2. Install necessary ROS2 Humble Packages
    ign_ros2_control
    ign_ros2_control_demos
    ros_gz_sim
    ros_gz_bridge
    teleop_tools
    rqt_image_view

3. !Adjust Path to world in launch file!
@TODO: Use relative package path in launch files
This needs to be adjusted: in launch/ackermann_drive_example.launch.py
            launch_arguments=[('gz_args', [' -r -v 4 **replace here your absoute path ros2_ws/src/ros2_ackermann_gazebo/world/emtpy_export.sdf**'])]),

4. Build and Source Workspace
    $colcon build & source install/setup.bash

5. Launch the Package
    $ros2 launch ros2_ackermann_gazebo ackermann_drive_example.launch.py

6. Launch the Teleop Keys
    $ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap cmd_vel:=/ackermann_steering_controller/reference -p stamped:=True


7. Inspect with RViz or rqt_image_view the camera feed
    $ros2 run rqt_image_view rqt_image_view


# Workspace Structure
```
├── launch
│   ├── ackermann_drive_example.launch.py
│   └── __pycache__
│       └── ackermann_drive_example.launch.cpython-310.pyc
├── LICENSE
├── package.xml
├── README.md
├── resource
│   └── ros2_ackermann_gazebo
├── ros2_ackermann_gazebo
│   └── __init__.py
├── setup.cfg
├── setup.py
├── sys_reqs (Packages installed on tested system)
│   ├── installed_ros2_packages.txt
│   └── sys_packages.txt
├── test
│   ├── test_copyright.py
│   ├── test_flake8.py
│   └── test_pep257.py
├── urdf
│   └── test_ackermann_drive.xacro.urdf (Robot Definition containing sensors + sensor locations (e. g. by setting camera_link))
└── world
    ├── emtpy_export.sdf (World containing the Racetrack texture as Ground)
    ├── racetrack.png (Racetrack Texture)
    └── racetrack.svg (Inkscape Graphic containing the Ractrack)
```


# Useful for Development

export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/example_racetrack/models

## Good Reference
https://github.com/arashsm79/ros-ign-gazebo-camera/tree/main/camera-sensor-example/launch


## Migration of Gazebo Classic to Igntition / Gazebo

https://gazebosim.org/api/gazebo/6/migrationsdf.html


## Old Texture Modeling -> see Migration Guide
https://classic.gazebosim.org/tutorials?tut=color_model

## Parameter Bridge
ros2 run ros_gz_bridge parameter_bridge image_raw@sensor_msgs/msg/Image@gz.msgs.Image

## Sensors Documentation

https://gazebosim.org/docs/fortress/sensors/
