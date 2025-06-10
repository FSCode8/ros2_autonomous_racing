from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, TimerAction

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=['src/planning2/config/controller_server.yaml'],
        ),
        TimerAction(
            period=2.0,
            actions=[
                ExecuteProcess(
                    cmd=['ros2', 'lifecycle', 'set', '/controller_server', 'configure'],
                    output='screen'
                )
            ]
        ),
        TimerAction(
            period=4.0,
            actions=[
                Node(
                    package='tf2_ros',
                    executable='static_transform_publisher',
                    name='static_transform_publisher',
                    arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_link'],
                    output='screen'
                )
            ]
        ),
        TimerAction(
            period=6.0,
            actions=[
                ExecuteProcess(
                    cmd=['ros2', 'lifecycle', 'set', '/controller_server', 'activate'],
                    output='screen'
                )
            ]
        ),
    ])