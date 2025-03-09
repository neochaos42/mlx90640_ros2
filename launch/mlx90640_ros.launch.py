from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='mlx90640_ros',
            executable='mlx90640_ros',
            name='mlx90640_ros',
            output='screen',
            parameters=[{'config_file': 'config/mlx90640_config.yaml'}]
        )
    ])