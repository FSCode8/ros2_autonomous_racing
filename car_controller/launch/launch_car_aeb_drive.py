from launch import LaunchDescription
from launch.actions import ExecuteProcess

def generate_launch_description():
    return LaunchDescription([
        ExecuteProcess(
            cmd=[
                'ros2', 'run', 'car_controller', 'drive',
                '--ros-args', '-p', 'use_sim_time:=true',
                '--params-file', 'src/car_controller/config/params_car.yaml'
            ],
            output='screen'
        ),
        ExecuteProcess(
            cmd=[
                'ros2', 'run', 'car_controller', 'aeb',
                '--ros-args', '-p', 'use_sim_time:=true',
                '--params-file', 'src/car_controller/config/params_car.yaml'
            ],
            output='screen'
        ),
    ])