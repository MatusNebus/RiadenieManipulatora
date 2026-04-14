from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    robot_description = Command([
        "xacro ",
        PathJoinSubstitution([
            FindPackageShare("abb_model"),
            "urdf",
            "abb_irb4600_60_205.xacro",
        ]),
    ])

    rviz_config = PathJoinSubstitution([
        FindPackageShare("pose_teacher"),
        "config",
        "pose_teacher.rviz",
    ])

    return LaunchDescription([
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[{"robot_description": robot_description}],
        ),
        Node(
            package="model_spawner",
            executable="model_spawner",
            name="model_spawner",
            output="screen",
        ),
        Node(
            package="pose_teacher",
            executable="pose_teacher",
            name="pose_teacher",
            output="screen",
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            parameters=[{"robot_description": robot_description}],
            arguments=["-d", rviz_config],
        ),
    ])
