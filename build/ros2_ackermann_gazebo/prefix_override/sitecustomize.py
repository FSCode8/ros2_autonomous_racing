import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/root/ros2_autonomous_racing/src/install/ros2_ackermann_gazebo'
