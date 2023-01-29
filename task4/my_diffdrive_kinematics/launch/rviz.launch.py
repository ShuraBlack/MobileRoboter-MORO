from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    base_path = os.path.realpath(
        get_package_share_directory('my_diffdrive_kinematics'))
    rviz_path = os.path.join(base_path, 'rviz', 'generate_path.rviz')

    arg_rviz_config = DeclareLaunchArgument('rviz_config',
                                            default_value=rviz_path,
                                            description='Default Path to Rviz '
                                                        'configuration to display Path.')

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', LaunchConfiguration('rviz_config')]
    )

    return LaunchDescription([
        arg_rviz_config,
        rviz_node
    ])
