from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="model_spawner",
            executable="model_spawner",
            name="model_spawner",
            output="screen",
        )
    ])
