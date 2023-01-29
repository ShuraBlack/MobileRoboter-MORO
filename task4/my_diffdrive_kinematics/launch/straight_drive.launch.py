from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    node = Node(
        package='my_diffdrive_kinematics',
        executable='straight_drive',
        name='straight_drive',
        parameters=[
            {"v": 0.2},
            {"distance": 2.0}
        ]
    )

    return LaunchDescription([
        node
    ])
