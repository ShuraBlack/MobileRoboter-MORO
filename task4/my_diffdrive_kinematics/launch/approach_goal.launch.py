from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    node = Node(
        package='my_diffdrive_kinematics',
        executable='approach_goal',
        name='approach_goal',
        parameters=[
            {"x": 1.5},
            {"y": 1.0},
            {"tol": 0.1},
            {"v_max": 0.25},
            {"omega_max": 2.5},
            {"k_v": 1.225},
            {"k_omega": 1.225},
        ]
    )

    return LaunchDescription([
        node
    ])
