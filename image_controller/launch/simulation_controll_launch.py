from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Run the teleop_twist_keyboard with remapping and parameters
    teleop_node = Node(
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        name='teleop',
        remappings=[
            ('cmd_vel', '/ackermann_steering_controller/reference'),
        ],
        parameters=[
            {'stamped': True}
        ],
        output='screen'
    )
    
    return LaunchDescription([
        teleop_node
    ])