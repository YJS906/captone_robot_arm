import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import LifecycleNode
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    ydlidar_share_dir = get_package_share_directory('ydlidar_ros2_driver')
    default_ydlidar_params_file = os.path.join(
        ydlidar_share_dir, 'params', 'ydlidar.yaml')

    base_port = LaunchConfiguration('base_port')
    lidar_port = LaunchConfiguration('lidar_port')
    use_base = LaunchConfiguration('use_base')
    publish_joint_states = LaunchConfiguration('publish_joint_states')
    ydlidar_params_file = LaunchConfiguration('ydlidar_params_file')
    cmd_vel_timeout_sec = LaunchConfiguration('cmd_vel_timeout_sec')
    left_angular_scale = LaunchConfiguration('left_angular_scale')
    right_angular_scale = LaunchConfiguration('right_angular_scale')

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
            default_value='true',
            description='Start LIMO base driver'),
        DeclareLaunchArgument(
            'publish_joint_states',
            default_value='false',
            description='Start LIMO joint_state_publisher for visual wheel joints'),
        DeclareLaunchArgument(
            'ydlidar_params_file',
            default_value=default_ydlidar_params_file,
            description='YDLiDAR parameter file'),
        DeclareLaunchArgument(
            'cmd_vel_timeout_sec',
            default_value='0.5',
            description='Stop the LIMO base when /cmd_vel is missing for this many seconds'),
        DeclareLaunchArgument(
            'left_angular_scale',
            default_value='1.0',
            description='Scale positive yaw odometry from the LIMO base'),
        DeclareLaunchArgument(
            'right_angular_scale',
            default_value='1.22',
            description='Scale negative yaw odometry from the LIMO base'),

        IncludeLaunchDescription(
            PathJoinSubstitution([
                FindPackageShare('limo_description'),
                'launch',
                'load_urdf.launch.py',
            ]),
            launch_arguments={
                'publish_joint_states': publish_joint_states,
            }.items()),

        IncludeLaunchDescription(
            PathJoinSubstitution([
                FindPackageShare('limo_base'),
                'launch',
                'limo_base.launch.py',
            ]),
            launch_arguments={
                'port_name': base_port,
                'pub_odom_tf': 'true',
                'cmd_vel_timeout_sec': cmd_vel_timeout_sec,
                'left_angular_scale': left_angular_scale,
                'right_angular_scale': right_angular_scale,
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
    ])
