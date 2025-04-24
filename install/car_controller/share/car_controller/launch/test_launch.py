from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='image_controller',
            executable='image_logic.py',
            name='logic'
        ),
        Node(
            package='car_controller',
            executable='ttc_node',
            name='ttc'
        ),
        Node(
            package='car_controller',
            executable='aeb_node',
            name='aeb'
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    get_package_share_directory('ros2_ackermann_gazebo'),
                    'launch',
                    'ackermann_drive_example.launch.py'
                ])
            )
        )
    ])
