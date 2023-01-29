from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    base_path = os.path.realpath(
        get_package_share_directory('my_diffdrive_kinematics'))
    rviz_path = os.path.join(base_path, 'rviz', 'generate_path.rviz')
    arg_config = DeclareLaunchArgument('config_file',
                                       default_value=os.path.join(
                                           base_path, 'config', 'default_path.yaml'),
                                       description='Source from where to load points for path.')

    arg_follow_config = DeclareLaunchArgument('follower_config_file',
                                              default_value=os.path.join(
                                                  base_path, 'config', 'follower_config.yaml'),
                                              description='Source from where to '
                                                          'load follower configuration.')

    arg_rviz = DeclareLaunchArgument('launch_rviz',
                                     default_value='False',
                                     description='Flag to start Rviz with node.')

    arg_gazebo = DeclareLaunchArgument('launch_gazebo',
                                       default_value='False',
                                       description='Flag to start Gazebo with node.')

    arg_rviz_config = DeclareLaunchArgument('rviz_config',
                                            default_value=rviz_path,
                                            description='Default Path to Rviz '
                                                        'configuration to display Path.')

    generate_path_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('my_diffdrive_kinematics'),
                         'launch', 'generate_path.launch.py')
        ),
        launch_arguments=[('config_file', LaunchConfiguration('config_file')),
                          ('launch_rviz', LaunchConfiguration('launch_rviz')),
                          ('launch_gazebo', LaunchConfiguration('launch_gazebo')),
                          ('rviz_config', LaunchConfiguration('rviz_config')), ]
    )

    follow_node = Node(
        package='my_diffdrive_kinematics',
        executable='line_follower',
        name='line_follower_node',
        parameters=[LaunchConfiguration('follower_config_file')]
    )

    return LaunchDescription([
        arg_config,
        arg_follow_config,
        arg_rviz,
        arg_gazebo,
        arg_rviz_config,
        generate_path_launch,
        follow_node
    ])
