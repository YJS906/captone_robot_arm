#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import AnyLaunchDescriptionSource, PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    camera_launch = LaunchConfiguration('camera_launch')
    start_camera = LaunchConfiguration('start_camera')
    start_detector = LaunchConfiguration('start_detector')
    start_grasp = LaunchConfiguration('start_grasp')
    start_moveit = LaunchConfiguration('start_moveit')
    start_hardware = LaunchConfiguration('start_hardware')
    port_name = LaunchConfiguration('port_name')

    camera_x = LaunchConfiguration('camera_x')
    camera_y = LaunchConfiguration('camera_y')
    camera_z = LaunchConfiguration('camera_z')
    camera_roll = LaunchConfiguration('camera_roll')
    camera_pitch = LaunchConfiguration('camera_pitch')
    camera_yaw = LaunchConfiguration('camera_yaw')
    base_frame = LaunchConfiguration('base_frame')
    camera_frame = LaunchConfiguration('camera_frame')
    execute_on_target = LaunchConfiguration('execute_on_target')
    auto_execute_cooldown_sec = LaunchConfiguration('auto_execute_cooldown_sec')
    arduino_serial_port = LaunchConfiguration('arduino_serial_port')
    arduino_baud = LaunchConfiguration('arduino_baud')
    start_arduino = LaunchConfiguration('start_arduino')
    use_uvc_camera = LaunchConfiguration('use_uvc_camera')
    uvc_product_id = LaunchConfiguration('uvc_product_id')

    pkg_share = get_package_share_directory('open_manipulator_vision_grasp')
    config_file = os.path.join(pkg_share, 'config', 'color_grasp.yaml')

    hardware_launch = os.path.join(
        get_package_share_directory('open_manipulator_x_bringup'),
        'launch',
        'hardware.launch.py',
    )
    moveit_launch = os.path.join(
        get_package_share_directory('open_manipulator_x_moveit_config'),
        'launch',
        'moveit_core.launch.py',
    )

    return LaunchDescription([
        DeclareLaunchArgument('camera_launch', default_value='dabai_dc1.launch.xml'),
        DeclareLaunchArgument('start_camera', default_value='true'),
        DeclareLaunchArgument('start_detector', default_value='true'),
        DeclareLaunchArgument('start_grasp', default_value='true'),
        DeclareLaunchArgument('start_moveit', default_value='true'),
        DeclareLaunchArgument('start_hardware', default_value='true'),
        DeclareLaunchArgument('port_name', default_value='/dev/ttyACM0'),
        DeclareLaunchArgument('base_frame', default_value='link1'),
        DeclareLaunchArgument('camera_frame', default_value='camera_link'),
        DeclareLaunchArgument('camera_x', default_value='0.00'),   #옆으로 +는 왼쪽 
        DeclareLaunchArgument('camera_y', default_value='-0.0204'),  # 앞으로  -는 앞으로 
        DeclareLaunchArgument('camera_z', default_value='-0.23'),
        DeclareLaunchArgument('camera_roll', default_value='0.0'),
        DeclareLaunchArgument('camera_pitch', default_value='0.0'),
        DeclareLaunchArgument('camera_yaw', default_value='-1.57079632679'),
        DeclareLaunchArgument('execute_on_target', default_value='false'),
        DeclareLaunchArgument('auto_execute_cooldown_sec', default_value='10.0'),
        DeclareLaunchArgument('arduino_serial_port', default_value='/dev/ttyACM1'),
        DeclareLaunchArgument('arduino_baud', default_value='9600'),
        DeclareLaunchArgument('start_arduino', default_value='true'),
        DeclareLaunchArgument('use_uvc_camera', default_value='true'),
        DeclareLaunchArgument('uvc_product_id', default_value='0x0557'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(hardware_launch),
            launch_arguments={'port_name': port_name}.items(),
            condition=IfCondition(start_hardware),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(moveit_launch),
            condition=IfCondition(start_moveit),
        ),
        IncludeLaunchDescription(
            AnyLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('astra_camera'),
                    'launch',
                    camera_launch,
                ])
            ),
            launch_arguments={
                'camera_name': 'camera',
                'enable_color': 'true',
                'enable_ir': 'false',
                'enable_point_cloud': 'false',
                'enable_colored_point_cloud': 'false',
                'use_uvc_camera': use_uvc_camera,
                'uvc_product_id': uvc_product_id,
            }.items(),
            condition=IfCondition(start_camera),
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='camera_to_arm_tf',
            arguments=[
                '--x', camera_x,
                '--y', camera_y,
                '--z', camera_z,
                '--roll', camera_roll,
                '--pitch', camera_pitch,
                '--yaw', camera_yaw,
                '--frame-id', base_frame,
                '--child-frame-id', camera_frame,
            ],
            condition=IfCondition(start_detector),
        ),
        Node(
            package='open_manipulator_vision_grasp',
            executable='detect_color_object.py',
            name='detect_color_object',
            output='screen',
            parameters=[config_file],
            condition=IfCondition(start_detector),
        ),
        Node(
            package='open_manipulator_vision_grasp',
            executable='arduino_metal_sensor.py',
            name='arduino_metal_sensor',
            output='screen',
            parameters=[
                config_file,
                {
                    'serial_port': arduino_serial_port,
                    'baudrate': arduino_baud,
                },
            ],
            condition=IfCondition(start_arduino),
        ),
        Node(
            package='open_manipulator_vision_grasp',
            executable='metal_grasp_coordinator.py',
            name='metal_grasp_coordinator',
            output='screen',
            parameters=[config_file],
            condition=IfCondition(start_grasp),
        ),
        Node(
            package='open_manipulator_vision_grasp',
            executable='color_grasp_moveit',
            name='color_grasp_moveit',
            output='screen',
            parameters=[
                config_file,
                {
                    'execute_on_target': execute_on_target,
                    'auto_execute_cooldown_sec': auto_execute_cooldown_sec,
                },
            ],
            condition=IfCondition(start_grasp),
        ),
    ])
