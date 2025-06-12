from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='perception',
            executable='image_bag_reader',
            name='bag_reader',
            parameters=[{'bag_filename': 'image_bag_work'}]
        ),
        Node(
            package='perception',
            executable='run_perception.py',
            name='logic'
        )
    ])