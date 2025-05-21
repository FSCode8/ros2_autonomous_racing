from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='image_controller',
            executable='image_bag_reader',
            name='bag_reader',
            parameters=[{'bag_filename': 'image_bag_work'}]
        ),
        Node(
            package='image_controller',
            executable='image_logic.py',
            name='logic'
        )
    ])