import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

from launch_ros.descriptions import ParameterFile

def generate_launch_description():
    wego_share_dir = get_package_share_directory('wego')
    wego_nav_share_dir = get_package_share_directory('wego_2d_nav')
    map_yaml_file = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')

    # setting for rviz configuration path
    rviz_file_name = 'navigation.rviz'
    rviz_config_path = os.path.join(wego_share_dir, 'rviz', rviz_file_name)

    declare_map_arg = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(wego_nav_share_dir, 'maps', 'map.yaml'),
        description='Full path to map yaml file')

    declare_params_arg = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(
            wego_nav_share_dir, 'params', 'diff_navigation_params.yaml'),
        description='Full path to Nav2 parameter file')

    # remapping tf topic
    remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]

    # set container for composable node
    nav2_container = Node(
        name='nav2_container',
        package='rclcpp_components',
        executable='component_container_isolated',
        parameters=[ParameterFile(params_file), {'autostart': True}],
        arguments=['--ros-args', '--log-level', 'info'],
        remappings=remappings,
        output='screen',
    )

    # For localization
    localization_launch = IncludeLaunchDescription(
        PathJoinSubstitution([
            FindPackageShare('wego_2d_nav'),
            'launch', 
            'localization_launch.py',
        ]),
        launch_arguments={
            'map' : map_yaml_file,
            'params_file': params_file
        }.items()
    )

    # For navigation
    navigation_launch = IncludeLaunchDescription(
        PathJoinSubstitution([
            FindPackageShare('wego_2d_nav'),
            'launch',
            'navigation_only_launch.py',
        ]),
        launch_arguments={
            'params_file': params_file
        }.items()
    )

    # setting for rviz
    rviz_config_node = Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_path],
        )

    return LaunchDescription([
        declare_map_arg,
        declare_params_arg,
        nav2_container,
        localization_launch,
        navigation_launch,
        rviz_config_node,
    ])
