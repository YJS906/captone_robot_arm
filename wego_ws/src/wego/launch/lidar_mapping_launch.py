import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import LifecycleNode, Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    wego_share_dir = get_package_share_directory('wego')
    ydlidar_share_dir = get_package_share_directory('ydlidar_ros2_driver')
    default_cartographer_config_dir = os.path.join(wego_share_dir, 'config')
    default_ydlidar_params_file = os.path.join(
        ydlidar_share_dir, 'params', 'ydlidar.yaml')

    base_port = LaunchConfiguration('base_port')
    lidar_port = LaunchConfiguration('lidar_port')
    use_base = LaunchConfiguration('use_base')
    use_ekf = LaunchConfiguration('use_ekf')
    use_rviz = LaunchConfiguration('rviz')
    resolution = LaunchConfiguration('resolution')
    publish_period_sec = LaunchConfiguration('publish_period_sec')

    cartographer_config_dir = LaunchConfiguration('cartographer_config_dir')
    configuration_file = LaunchConfiguration('configuration_file')
    ydlidar_params_file = LaunchConfiguration('ydlidar_params_file')

    rviz_config_path = os.path.join(wego_share_dir, 'rviz', 'cartographer.rviz')

    return LaunchDescription([
        DeclareLaunchArgument(
            'base_port',
            default_value='ttyTHS0',
            description='LIMO base serial device name without /dev/'),
        DeclareLaunchArgument(
            'lidar_port',
            default_value='/dev/ttyUSB0',
            description='YDLiDAR serial device path'),
        DeclareLaunchArgument(
            'use_base',
            default_value='false',
            description='Start LIMO base driver for ROS teleop mapping'),
        DeclareLaunchArgument(
            'use_ekf',
            default_value='false',
            description='Start robot_localization EKF; not needed for lidar-only SLAM'),
        DeclareLaunchArgument(
            'ydlidar_params_file',
            default_value=default_ydlidar_params_file,
            description='YDLiDAR parameter file'),
        DeclareLaunchArgument(
            'cartographer_config_dir',
            default_value=default_cartographer_config_dir,
            description='Cartographer configuration directory'),
        DeclareLaunchArgument(
            'configuration_file',
            default_value='limo_lds_2d_lidar_only.lua',
            description='Cartographer lua configuration file'),
        DeclareLaunchArgument(
            'resolution',
            default_value='0.05',
            description='Occupancy grid resolution'),
        DeclareLaunchArgument(
            'publish_period_sec',
            default_value='1.0',
            description='Occupancy grid publish period'),
        DeclareLaunchArgument(
            'rviz',
            default_value='true',
            description='Start RViz with the Cartographer view'),

        IncludeLaunchDescription(
            PathJoinSubstitution([
                FindPackageShare('limo_description'),
                'launch',
                'load_urdf.launch.py',
            ])),

        IncludeLaunchDescription(
            PathJoinSubstitution([
                FindPackageShare('limo_base'),
                'launch',
                'limo_base.launch.py',
            ]),
            launch_arguments={
                'port_name': base_port,
                'pub_odom_tf': 'False',
            }.items(),
            condition=IfCondition(use_base)),

        LifecycleNode(
            package='ydlidar_ros2_driver',
            executable='ydlidar_ros2_driver_node',
            name='ydlidar_ros2_driver_node',
            namespace='/',
            output='screen',
            emulate_tty=True,
            parameters=[
                ydlidar_params_file,
                {'port': lidar_port},
            ]),

        IncludeLaunchDescription(
            PathJoinSubstitution([
                FindPackageShare('robot_localization'),
                'launch',
                'limo_ekf_launch.py',
            ]),
            condition=IfCondition(use_ekf)),

        Node(
            package='cartographer_ros',
            executable='cartographer_node',
            name='cartographer_node',
            output='screen',
            remappings=[('odom', 'odometry/filtered')],
            arguments=[
                '-configuration_directory',
                cartographer_config_dir,
                '-configuration_basename',
                configuration_file,
            ]),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('wego'),
                    'launch',
                    'occupancy_grid_launch.py',
                ])),
            launch_arguments={
                'resolution': resolution,
                'publish_period_sec': publish_period_sec,
            }.items()),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            condition=IfCondition(use_rviz),
            arguments=['-d', rviz_config_path]),
    ])
