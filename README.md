# ROS2 Autonomous Racing Project 
## Get the project up and running (Linux) 
cd ~  
mkdir ros2_autonomous_racing  
cd ros2_autonomous_racing  
mkdir src  
cd src  
git clone git@github.com:FSCode8/ros2_autonomous_racing.git .  

-> change file path in ros2_ackermann_gazebo/launch/ackermann_drive_example.launch.py  
-> change file path in ros2_ackermann_gazebo/world/emtpy_export.sdf  

cd ~/ros2_autonomous_racing  
colcon build --packages-select ros2_ackermann_gazebo

sudo rosdep init  
rosdep update  
rosdep install --from-paths src -y --ignore-src  

-> gazebo simulation should be executable in new terminal by doing:  
cd ~/ros2_autonomous_racing  
. install/setup.bash  
ros2 launch ros2_ackermann_gazebo ackermann_drive_example.launch.py  
