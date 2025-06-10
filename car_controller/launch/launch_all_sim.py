from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    ackermann_launch = os.path.join(
        get_package_share_directory('ros2_ackermann_gazebo'),
        'launch',
        'ackermann_drive_example.launch.py'
    )
    nav2_controller_launch = os.path.join(
        get_package_share_directory('nav2_controller'),
        'launch',
        'sim_planning.py'
    )
    aeb_and_drive_launch = os.path.join(
        get_package_share_directory('car_controller'),
        'launch',
        'launch_sim_aeb_drive.py'
    )

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([ackermann_launch])
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([nav2_controller_launch])
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([aeb_and_drive_launch])
        ),
        ExecuteProcess(
            cmd=[
                'ros2', 'run', 'image_controller', 'image_logic.py',
                '--ros-args', '-p', 'use_sim_time:=true'
            ],
            output='screen'
        ),
    ])