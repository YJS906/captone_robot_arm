from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    coverage_config_file = LaunchConfiguration('coverage_config_file')
    coverage_cell_size = LaunchConfiguration('coverage_cell_size')
    start_automatically = LaunchConfiguration('start_automatically')
    unknown_is_blocked = LaunchConfiguration('unknown_is_blocked')
    occupied_threshold = LaunchConfiguration('occupied_threshold')
    map_topic = LaunchConfiguration('map_topic')
    face_next_goal = LaunchConfiguration('face_next_goal')
    path_strategy = LaunchConfiguration('path_strategy')
    stripe_axis = LaunchConfiguration('stripe_axis')
    execution_mode = LaunchConfiguration('execution_mode')
    robot_base_frame = LaunchConfiguration('robot_base_frame')
    cmd_vel_topic = LaunchConfiguration('cmd_vel_topic')
    linear_speed = LaunchConfiguration('linear_speed')
    lookahead_distance = LaunchConfiguration('lookahead_distance')
    max_angular_speed = LaunchConfiguration('max_angular_speed')
    goal_tolerance = LaunchConfiguration('goal_tolerance')
    straight_goal_tolerance = LaunchConfiguration('straight_goal_tolerance')
    corner_goal_tolerance = LaunchConfiguration('corner_goal_tolerance')
    return_goal_tolerance = LaunchConfiguration('return_goal_tolerance')
    corner_angle_threshold = LaunchConfiguration('corner_angle_threshold')
    yaw_tolerance = LaunchConfiguration('yaw_tolerance')
    rotate_to_path_yaw = LaunchConfiguration('rotate_to_path_yaw')
    enable_reverse_goal_recovery = LaunchConfiguration('enable_reverse_goal_recovery')
    reverse_speed = LaunchConfiguration('reverse_speed')
    reverse_goal_recovery_distance = LaunchConfiguration('reverse_goal_recovery_distance')
    path_interpolation_resolution = LaunchConfiguration('path_interpolation_resolution')
    coverage_boundary_margin = LaunchConfiguration('coverage_boundary_margin')
    corner_mode = LaunchConfiguration('corner_mode')
    corner_points = LaunchConfiguration('corner_points')
    clicked_corner_topic = LaunchConfiguration('clicked_corner_topic')
    require_free_cell_center = LaunchConfiguration('require_free_cell_center')
    safe_zone_path_index = LaunchConfiguration('safe_zone_path_index')
    safe_zone_point = LaunchConfiguration('safe_zone_point')
    start_direction_mode = LaunchConfiguration('start_direction_mode')
    require_start_in_safe_zone = LaunchConfiguration('require_start_in_safe_zone')
    safe_zone_start_tolerance = LaunchConfiguration('safe_zone_start_tolerance')
    reset_to_safe_zone_on_start = LaunchConfiguration('reset_to_safe_zone_on_start')
    return_to_safe_zone_on_complete = LaunchConfiguration('return_to_safe_zone_on_complete')
    return_to_safe_zone_on_mine = LaunchConfiguration('return_to_safe_zone_on_mine')
    simulated_mine_path_indices = LaunchConfiguration('simulated_mine_path_indices')
    mine_detection_mode = LaunchConfiguration('mine_detection_mode')
    metal_events_topic = LaunchConfiguration('metal_events_topic')
    metal_state_topic = LaunchConfiguration('metal_state_topic')
    metal_event_keywords = LaunchConfiguration('metal_event_keywords')
    arm_sequence_service = LaunchConfiguration('arm_sequence_service')
    arm_sequence_service_timeout = LaunchConfiguration('arm_sequence_service_timeout')
    arm_drop_service = LaunchConfiguration('arm_drop_service')
    arm_drop_service_timeout = LaunchConfiguration('arm_drop_service_timeout')
    metal_backup_distance = LaunchConfiguration('metal_backup_distance')
    metal_backup_speed = LaunchConfiguration('metal_backup_speed')
    continue_after_arm_failure = LaunchConfiguration('continue_after_arm_failure')
    mission_status_topic = LaunchConfiguration('mission_status_topic')
    scan_topic = LaunchConfiguration('scan_topic')
    enable_obstacle_stop = LaunchConfiguration('enable_obstacle_stop')
    front_obstacle_stop_distance = LaunchConfiguration('front_obstacle_stop_distance')
    front_obstacle_angle = LaunchConfiguration('front_obstacle_angle')

    return LaunchDescription([
        DeclareLaunchArgument(
            'coverage_config_file',
            default_value='',
            description='Optional YAML file describing coverage grid corners and options'),

        DeclareLaunchArgument(
            'coverage_cell_size',
            default_value='0.5',
            description='World size in meters for one coverage grid cell'),

        DeclareLaunchArgument(
            'start_automatically',
            default_value='false',
            description='Start sending Nav2 goals as soon as the map is converted'),

        DeclareLaunchArgument(
            'unknown_is_blocked',
            default_value='true',
            description='Treat unknown occupancy values as blocked cells'),

        DeclareLaunchArgument(
            'occupied_threshold',
            default_value='50',
            description='Occupancy value considered blocked'),

        DeclareLaunchArgument(
            'map_topic',
            default_value='/map',
            description='Input nav_msgs/OccupancyGrid topic'),

        DeclareLaunchArgument(
            'face_next_goal',
            default_value='true',
            description='Set each goal orientation toward the next coverage cell'),

        DeclareLaunchArgument(
            'path_strategy',
            default_value='cell_centers',
            description='Coverage goal strategy: stripe_endpoints or cell_centers'),

        DeclareLaunchArgument(
            'stripe_axis',
            default_value='x',
            description='Stripe direction: x for left/right rows, y for up/down columns'),

        DeclareLaunchArgument(
            'execution_mode',
            default_value='path_follower',
            description='Execution mode: path_follower or nav2'),

        DeclareLaunchArgument(
            'robot_base_frame',
            default_value='base_link',
            description='Robot base frame used by the path follower'),

        DeclareLaunchArgument(
            'cmd_vel_topic',
            default_value='/cmd_vel',
            description='Velocity command topic for path_follower mode'),

        DeclareLaunchArgument(
            'linear_speed',
            default_value='0.15',
            description='Constant forward speed in path_follower mode'),

        DeclareLaunchArgument(
            'lookahead_distance',
            default_value='0.45',
            description='Pure pursuit lookahead distance in meters'),

        DeclareLaunchArgument(
            'max_angular_speed',
            default_value='1.0',
            description='Maximum angular speed in rad/s'),

        DeclareLaunchArgument(
            'goal_tolerance',
            default_value='0.15',
            description='Fallback distance tolerance for each path point'),

        DeclareLaunchArgument(
            'straight_goal_tolerance',
            default_value='-1.0',
            description='Distance tolerance for straight path points; <=0 uses goal_tolerance'),

        DeclareLaunchArgument(
            'corner_goal_tolerance',
            default_value='-1.0',
            description='Distance tolerance for corner path points; <=0 uses goal_tolerance'),

        DeclareLaunchArgument(
            'return_goal_tolerance',
            default_value='0.08',
            description='Distance tolerance for A* safe-zone return path points'),

        DeclareLaunchArgument(
            'corner_angle_threshold',
            default_value='0.5',
            description='Heading change in radians used to classify a path point as a corner'),

        DeclareLaunchArgument(
            'yaw_tolerance',
            default_value='0.08',
            description='Yaw tolerance for in-place rotation'),

        DeclareLaunchArgument(
            'rotate_to_path_yaw',
            default_value='0.35',
            description='Yaw error threshold that triggers in-place rotation'),

        DeclareLaunchArgument(
            'enable_reverse_goal_recovery',
            default_value='true',
            description='Reverse slowly when the current goal point is missed behind the robot'),

        DeclareLaunchArgument(
            'reverse_speed',
            default_value='0.04',
            description='Reverse speed in meters per second for missed-goal recovery'),

        DeclareLaunchArgument(
            'reverse_goal_recovery_distance',
            default_value='0.25',
            description='Maximum distance to use reverse recovery for a missed goal point'),

        DeclareLaunchArgument(
            'path_interpolation_resolution',
            default_value='0.05',
            description='Published coverage path interpolation spacing in meters'),

        DeclareLaunchArgument(
            'coverage_boundary_margin',
            default_value='0.2',
            description='Inset coverage grid from selected rectangle edges in meters'),

        DeclareLaunchArgument(
            'corner_mode',
            default_value='clicked',
            description='Coverage rectangle source: clicked, param, or auto_pca'),

        DeclareLaunchArgument(
            'corner_points',
            default_value='',
            description='CSV corner points in map frame: x1,y1,x2,y2,x3,y3,x4,y4'),

        DeclareLaunchArgument(
            'clicked_corner_topic',
            default_value='/clicked_point',
            description='geometry_msgs/PointStamped topic used to select 4 coverage corners'),

        DeclareLaunchArgument(
            'require_free_cell_center',
            default_value='false',
            description='If true, only grid centers that are free in /map become goals'),

        DeclareLaunchArgument(
            'safe_zone_path_index',
            default_value='1',
            description='1-based coverage path index used as the safe-zone start cell'),

        DeclareLaunchArgument(
            'safe_zone_point',
            default_value='',
            description='Map-frame safe-zone point "x,y"; used to choose forward or reverse path order'),

        DeclareLaunchArgument(
            'start_direction_mode',
            default_value='safe_zone_nearest_corner',
            description='Path start policy: path_order, safe_zone_nearest_endpoint, or safe_zone_nearest_corner'),

        DeclareLaunchArgument(
            'require_start_in_safe_zone',
            default_value='true',
            description='Require robot pose to be near the safe zone before starting coverage'),

        DeclareLaunchArgument(
            'safe_zone_start_tolerance',
            default_value='0.25',
            description='Allowed distance from safe-zone center before start in meters'),

        DeclareLaunchArgument(
            'reset_to_safe_zone_on_start',
            default_value='true',
            description='Reset current coverage index to the safe zone when start_coverage is called'),

        DeclareLaunchArgument(
            'return_to_safe_zone_on_complete',
            default_value='true',
            description='Return to the safe zone after all coverage goals are visited'),

        DeclareLaunchArgument(
            'return_to_safe_zone_on_mine',
            default_value='true',
            description='Return to the safe zone after a detected mine is handled'),

        DeclareLaunchArgument(
            'simulated_mine_path_indices',
            default_value='',
            description='Comma-separated 1-based coverage path numbers that simulate mine detection'),

        DeclareLaunchArgument(
            'mine_detection_mode',
            default_value='sensor',
            description='Mine detection source: sensor, simulated, or both'),

        DeclareLaunchArgument(
            'metal_events_topic',
            default_value='/arduino/events',
            description='String topic published by Arduino metal sensor node'),

        DeclareLaunchArgument(
            'metal_state_topic',
            default_value='/metal_sensor/any_detected',
            description='Bool topic that is true when any metal sensor is detected'),

        DeclareLaunchArgument(
            'metal_event_keywords',
            default_value='METAL,metal_detected,detected',
            description='Comma-separated keywords that classify an Arduino event as metal detected'),

        DeclareLaunchArgument(
            'arm_sequence_service',
            default_value='/run_metal_grasp_sequence',
            description='std_srvs/Trigger service that runs blower and grasp sequence'),

        DeclareLaunchArgument(
            'arm_sequence_service_timeout',
            default_value='120.0',
            description='Timeout in seconds for arm/blower/grasp sequence'),

        DeclareLaunchArgument(
            'arm_drop_service',
            default_value='/drop_mine_at_safe_zone',
            description='std_srvs/Trigger service that drops the carried mine at the safe zone'),

        DeclareLaunchArgument(
            'arm_drop_service_timeout',
            default_value='120.0',
            description='Timeout in seconds for safe-zone arm drop sequence'),

        DeclareLaunchArgument(
            'metal_backup_distance',
            default_value='0.08',
            description='Reverse distance before arm sequence after metal detection'),

        DeclareLaunchArgument(
            'metal_backup_speed',
            default_value='0.03',
            description='Reverse speed before arm sequence after metal detection'),

        DeclareLaunchArgument(
            'continue_after_arm_failure',
            default_value='false',
            description='If true, return to safe zone even if arm sequence fails'),

        DeclareLaunchArgument(
            'mission_status_topic',
            default_value='/coverage_mission_status',
            description='String status topic for coverage/metal/arm integration'),

        DeclareLaunchArgument(
            'scan_topic',
            default_value='/scan',
            description='LaserScan topic used for obstacle stop in path_follower mode'),

        DeclareLaunchArgument(
            'enable_obstacle_stop',
            default_value='true',
            description='Stop path_follower when a front obstacle is too close'),

        DeclareLaunchArgument(
            'front_obstacle_stop_distance',
            default_value='0.25',
            description='Front obstacle stop distance in meters'),

        DeclareLaunchArgument(
            'front_obstacle_angle',
            default_value='0.8',
            description='Front obstacle check angle in radians'),

        Node(
            package='wego',
            executable='coverage_path_planner.py',
            name='coverage_path_planner',
            output='screen',
            parameters=[{
                'coverage_config_file': ParameterValue(coverage_config_file, value_type=str),
                'coverage_cell_size': ParameterValue(coverage_cell_size, value_type=float),
                'start_automatically': ParameterValue(start_automatically, value_type=bool),
                'unknown_is_blocked': ParameterValue(unknown_is_blocked, value_type=bool),
                'occupied_threshold': ParameterValue(occupied_threshold, value_type=int),
                'map_topic': ParameterValue(map_topic, value_type=str),
                'face_next_goal': ParameterValue(face_next_goal, value_type=bool),
                'path_strategy': ParameterValue(path_strategy, value_type=str),
                'stripe_axis': ParameterValue(stripe_axis, value_type=str),
                'execution_mode': ParameterValue(execution_mode, value_type=str),
                'robot_base_frame': ParameterValue(robot_base_frame, value_type=str),
                'cmd_vel_topic': ParameterValue(cmd_vel_topic, value_type=str),
                'linear_speed': ParameterValue(linear_speed, value_type=float),
                'lookahead_distance': ParameterValue(lookahead_distance, value_type=float),
                'max_angular_speed': ParameterValue(max_angular_speed, value_type=float),
                'goal_tolerance': ParameterValue(goal_tolerance, value_type=float),
                'straight_goal_tolerance': ParameterValue(
                    straight_goal_tolerance, value_type=float),
                'corner_goal_tolerance': ParameterValue(
                    corner_goal_tolerance, value_type=float),
                'return_goal_tolerance': ParameterValue(
                    return_goal_tolerance, value_type=float),
                'corner_angle_threshold': ParameterValue(
                    corner_angle_threshold, value_type=float),
                'yaw_tolerance': ParameterValue(yaw_tolerance, value_type=float),
                'rotate_to_path_yaw': ParameterValue(rotate_to_path_yaw, value_type=float),
                'enable_reverse_goal_recovery': ParameterValue(
                    enable_reverse_goal_recovery, value_type=bool),
                'reverse_speed': ParameterValue(reverse_speed, value_type=float),
                'reverse_goal_recovery_distance': ParameterValue(
                    reverse_goal_recovery_distance, value_type=float),
                'path_interpolation_resolution': ParameterValue(
                    path_interpolation_resolution, value_type=float),
                'coverage_boundary_margin': ParameterValue(
                    coverage_boundary_margin, value_type=float),
                'corner_mode': ParameterValue(corner_mode, value_type=str),
                'corner_points': ParameterValue(corner_points, value_type=str),
                'clicked_corner_topic': ParameterValue(clicked_corner_topic, value_type=str),
                'require_free_cell_center': ParameterValue(require_free_cell_center, value_type=bool),
                'safe_zone_path_index': ParameterValue(safe_zone_path_index, value_type=int),
                'safe_zone_point': ParameterValue(safe_zone_point, value_type=str),
                'start_direction_mode': ParameterValue(start_direction_mode, value_type=str),
                'require_start_in_safe_zone': ParameterValue(
                    require_start_in_safe_zone, value_type=bool),
                'safe_zone_start_tolerance': ParameterValue(
                    safe_zone_start_tolerance, value_type=float),
                'reset_to_safe_zone_on_start': ParameterValue(
                    reset_to_safe_zone_on_start, value_type=bool),
                'return_to_safe_zone_on_complete': ParameterValue(
                    return_to_safe_zone_on_complete, value_type=bool),
                'return_to_safe_zone_on_mine': ParameterValue(
                    return_to_safe_zone_on_mine, value_type=bool),
                'simulated_mine_path_indices': ParameterValue(
                    simulated_mine_path_indices, value_type=str),
                'mine_detection_mode': ParameterValue(mine_detection_mode, value_type=str),
                'metal_events_topic': ParameterValue(metal_events_topic, value_type=str),
                'metal_state_topic': ParameterValue(metal_state_topic, value_type=str),
                'metal_event_keywords': ParameterValue(metal_event_keywords, value_type=str),
                'arm_sequence_service': ParameterValue(arm_sequence_service, value_type=str),
                'arm_sequence_service_timeout': ParameterValue(
                    arm_sequence_service_timeout, value_type=float),
                'arm_drop_service': ParameterValue(arm_drop_service, value_type=str),
                'arm_drop_service_timeout': ParameterValue(arm_drop_service_timeout, value_type=float),
                'metal_backup_distance': ParameterValue(metal_backup_distance, value_type=float),
                'metal_backup_speed': ParameterValue(metal_backup_speed, value_type=float),
                'continue_after_arm_failure': ParameterValue(
                    continue_after_arm_failure, value_type=bool),
                'mission_status_topic': ParameterValue(mission_status_topic, value_type=str),
                'scan_topic': ParameterValue(scan_topic, value_type=str),
                'enable_obstacle_stop': ParameterValue(enable_obstacle_stop, value_type=bool),
                'front_obstacle_stop_distance': ParameterValue(
                    front_obstacle_stop_distance, value_type=float),
                'front_obstacle_angle': ParameterValue(front_obstacle_angle, value_type=float),
            }]),
    ])
