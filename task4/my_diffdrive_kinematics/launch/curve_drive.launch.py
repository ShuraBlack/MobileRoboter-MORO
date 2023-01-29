from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    node = Node(
        package='my_diffdrive_kinematics',
        executable='curve_drive',
        name='curve_drive',
        parameters=[
            {"v": 0.2},
            {"r": 1.0},
            {"alpha": 1.2},
        ]
    )

    return LaunchDescription([
        node
    ])
