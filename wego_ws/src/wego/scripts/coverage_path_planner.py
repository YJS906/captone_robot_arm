#!/usr/bin/env python3

import math
import heapq
from enum import IntEnum

import rclpy
import yaml
from geometry_msgs.msg import Point, PointStamped, PoseStamped, Twist
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid, Path
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String
from action_msgs.msg import GoalStatus
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray


class CellState(IntEnum):
    BLOCKED = 100
    UNVISITED = 0
    VISITED = 50


class CoveragePathPlanner(Node):
    def __init__(self):
        super().__init__('coverage_path_planner')

        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('grid_occupancy_topic', '/grid_occupancy')
        self.declare_parameter('coverage_path_topic', '/coverage_path')
        self.declare_parameter('grid_marker_topic', '/coverage_grid_markers')
        self.declare_parameter('priority_marker_topic', '/coverage_priority_markers')
        self.declare_parameter('coverage_config_file', '')
        self.declare_parameter('coverage_cell_size', 0.5)
        self.declare_parameter('occupied_threshold', 50)
        self.declare_parameter('unknown_is_blocked', True)
        self.declare_parameter('start_automatically', False)
        self.declare_parameter('goal_frame_id', 'map')
        self.declare_parameter('goal_timeout_sec', 120.0)
        self.declare_parameter('face_next_goal', True)
        self.declare_parameter('path_strategy', 'cell_centers')
        self.declare_parameter('stripe_axis', 'x')
        self.declare_parameter('execution_mode', 'path_follower')
        self.declare_parameter('robot_base_frame', 'base_link')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('control_frequency', 20.0)
        self.declare_parameter('linear_speed', 0.15)
        self.declare_parameter('lookahead_distance', 0.45)
        self.declare_parameter('max_angular_speed', 1.0)
        self.declare_parameter('goal_tolerance', 0.15)
        self.declare_parameter('straight_goal_tolerance', -1.0)
        self.declare_parameter('corner_goal_tolerance', -1.0)
        self.declare_parameter('return_goal_tolerance', 0.08)
        self.declare_parameter('corner_angle_threshold', 0.5)
        self.declare_parameter('yaw_tolerance', 0.08)
        self.declare_parameter('rotate_to_path_yaw', 0.35)
        self.declare_parameter('enable_reverse_goal_recovery', True)
        self.declare_parameter('reverse_speed', 0.04)
        self.declare_parameter('reverse_goal_recovery_distance', 0.25)
        self.declare_parameter('path_interpolation_resolution', 0.05)
        self.declare_parameter('coverage_boundary_margin', 0.2)
        self.declare_parameter('corner_mode', 'clicked')
        self.declare_parameter('clicked_corner_topic', '/clicked_point')
        self.declare_parameter('corner_points', '')
        self.declare_parameter('require_free_cell_center', False)
        self.declare_parameter('safe_zone_path_index', 1)
        self.declare_parameter('safe_zone_point', '')
        self.declare_parameter('start_direction_mode', 'safe_zone_nearest_endpoint')
        self.declare_parameter('require_start_in_safe_zone', True)
        self.declare_parameter('safe_zone_start_tolerance', 0.25)
        self.declare_parameter('reset_to_safe_zone_on_start', True)
        self.declare_parameter('return_to_safe_zone_on_complete', True)
        self.declare_parameter('return_to_safe_zone_on_mine', True)
        self.declare_parameter('safe_zone_drop_yaw_rad', 0.0)
        self.declare_parameter('simulated_mine_path_indices', '')
        self.declare_parameter('mine_detection_mode', 'sensor')
        self.declare_parameter('metal_events_topic', '/arduino/events')
        self.declare_parameter('metal_state_topic', '/metal_sensor/any_detected')
        self.declare_parameter('metal_event_keywords', 'METAL,metal_detected,detected')
        self.declare_parameter('arm_sequence_service', '/run_metal_grasp_sequence')
        self.declare_parameter('arm_sequence_service_timeout', 120.0)
        self.declare_parameter('arm_drop_service', '/drop_mine_at_safe_zone')
        self.declare_parameter('arm_drop_service_timeout', 120.0)
        self.declare_parameter('metal_backup_distance', 0.05)
        self.declare_parameter('metal_backup_speed', 0.03)
        self.declare_parameter('continue_after_arm_failure', False)
        self.declare_parameter('mission_status_topic', '/coverage_mission_status')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('enable_obstacle_stop', True)
        self.declare_parameter('front_obstacle_stop_distance', 0.25)
        self.declare_parameter('front_obstacle_angle', 0.8)

        self.map_topic = self.get_parameter('map_topic').value
        self.grid_topic = self.get_parameter('grid_occupancy_topic').value
        self.path_topic = self.get_parameter('coverage_path_topic').value
        self.grid_marker_topic = self.get_parameter('grid_marker_topic').value
        self.priority_marker_topic = self.get_parameter('priority_marker_topic').value
        self.coverage_config_file = self.get_parameter('coverage_config_file').value
        self.coverage_cell_size = float(self.get_parameter('coverage_cell_size').value)
        self.occupied_threshold = int(self.get_parameter('occupied_threshold').value)
        self.unknown_is_blocked = bool(self.get_parameter('unknown_is_blocked').value)
        self.start_automatically = bool(self.get_parameter('start_automatically').value)
        self.goal_frame_id = self.get_parameter('goal_frame_id').value
        self.goal_timeout_sec = float(self.get_parameter('goal_timeout_sec').value)
        self.face_next_goal = bool(self.get_parameter('face_next_goal').value)
        self.path_strategy = self.get_parameter('path_strategy').value
        self.stripe_axis = self.get_parameter('stripe_axis').value
        self.execution_mode = self.get_parameter('execution_mode').value
        self.robot_base_frame = self.get_parameter('robot_base_frame').value
        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self.control_frequency = float(self.get_parameter('control_frequency').value)
        self.linear_speed = float(self.get_parameter('linear_speed').value)
        self.lookahead_distance = float(self.get_parameter('lookahead_distance').value)
        self.max_angular_speed = float(self.get_parameter('max_angular_speed').value)
        self.goal_tolerance = float(self.get_parameter('goal_tolerance').value)
        self.straight_goal_tolerance = float(
            self.get_parameter('straight_goal_tolerance').value)
        self.corner_goal_tolerance = float(
            self.get_parameter('corner_goal_tolerance').value)
        self.return_goal_tolerance = float(
            self.get_parameter('return_goal_tolerance').value)
        self.corner_angle_threshold = float(
            self.get_parameter('corner_angle_threshold').value)
        self.yaw_tolerance = float(self.get_parameter('yaw_tolerance').value)
        self.rotate_to_path_yaw = float(self.get_parameter('rotate_to_path_yaw').value)
        self.enable_reverse_goal_recovery = bool(
            self.get_parameter('enable_reverse_goal_recovery').value)
        self.reverse_speed = float(self.get_parameter('reverse_speed').value)
        self.reverse_goal_recovery_distance = float(
            self.get_parameter('reverse_goal_recovery_distance').value)
        self.path_interpolation_resolution = float(
            self.get_parameter('path_interpolation_resolution').value)
        self.coverage_boundary_margin = float(
            self.get_parameter('coverage_boundary_margin').value)
        self.corner_mode = self.get_parameter('corner_mode').value
        self.clicked_corner_topic = self.get_parameter('clicked_corner_topic').value
        self.corner_points = self.get_parameter('corner_points').value
        self.require_free_cell_center = bool(self.get_parameter('require_free_cell_center').value)
        self.safe_zone_path_index = int(self.get_parameter('safe_zone_path_index').value)
        self.safe_zone_point = self.get_parameter('safe_zone_point').value
        self.start_direction_mode = self.get_parameter('start_direction_mode').value
        self.require_start_in_safe_zone = bool(
            self.get_parameter('require_start_in_safe_zone').value)
        self.safe_zone_start_tolerance = float(
            self.get_parameter('safe_zone_start_tolerance').value)
        self.reset_to_safe_zone_on_start = bool(
            self.get_parameter('reset_to_safe_zone_on_start').value)
        self.return_to_safe_zone_on_complete = bool(
            self.get_parameter('return_to_safe_zone_on_complete').value)
        self.return_to_safe_zone_on_mine = bool(
            self.get_parameter('return_to_safe_zone_on_mine').value)
        self.safe_zone_drop_yaw_rad = float(
            self.get_parameter('safe_zone_drop_yaw_rad').value)
        self.simulated_mine_path_indices = self.parse_int_set(
            self.get_parameter('simulated_mine_path_indices').value)
        self.mine_detection_mode = self.get_parameter('mine_detection_mode').value
        self.metal_events_topic = self.get_parameter('metal_events_topic').value
        self.metal_state_topic = self.get_parameter('metal_state_topic').value
        self.metal_event_keywords = self.parse_string_list(
            self.get_parameter('metal_event_keywords').value)
        self.arm_sequence_service = self.get_parameter('arm_sequence_service').value
        self.arm_sequence_service_timeout = float(
            self.get_parameter('arm_sequence_service_timeout').value)
        self.arm_drop_service = self.get_parameter('arm_drop_service').value
        self.arm_drop_service_timeout = float(
            self.get_parameter('arm_drop_service_timeout').value)
        self.metal_backup_distance = float(self.get_parameter('metal_backup_distance').value)
        self.metal_backup_speed = float(self.get_parameter('metal_backup_speed').value)
        self.continue_after_arm_failure = bool(
            self.get_parameter('continue_after_arm_failure').value)
        self.mission_status_topic = self.get_parameter('mission_status_topic').value
        self.scan_topic = self.get_parameter('scan_topic').value
        self.enable_obstacle_stop = bool(self.get_parameter('enable_obstacle_stop').value)
        self.front_obstacle_stop_distance = float(
            self.get_parameter('front_obstacle_stop_distance').value)
        self.front_obstacle_angle = float(self.get_parameter('front_obstacle_angle').value)
        self.load_coverage_config()
        if self.straight_goal_tolerance <= 0.0:
            self.straight_goal_tolerance = self.goal_tolerance
        if self.corner_goal_tolerance <= 0.0:
            self.corner_goal_tolerance = self.goal_tolerance

        if self.coverage_cell_size <= 0.0:
            raise ValueError('coverage_cell_size must be greater than 0.0')
        if self.control_frequency <= 0.0:
            raise ValueError('control_frequency must be greater than 0.0')
        if self.path_interpolation_resolution <= 0.0:
            raise ValueError('path_interpolation_resolution must be greater than 0.0')
        if self.straight_goal_tolerance <= 0.0:
            raise ValueError('straight_goal_tolerance must be greater than 0.0')
        if self.corner_goal_tolerance <= 0.0:
            raise ValueError('corner_goal_tolerance must be greater than 0.0')
        if self.return_goal_tolerance <= 0.0:
            raise ValueError('return_goal_tolerance must be greater than 0.0')
        if self.corner_angle_threshold <= 0.0:
            raise ValueError('corner_angle_threshold must be greater than 0.0')
        if self.reverse_speed <= 0.0:
            raise ValueError('reverse_speed must be greater than 0.0')
        if self.reverse_goal_recovery_distance <= 0.0:
            raise ValueError('reverse_goal_recovery_distance must be greater than 0.0')
        if self.safe_zone_path_index <= 0:
            raise ValueError('safe_zone_path_index must be greater than 0')
        if self.safe_zone_start_tolerance <= 0.0:
            raise ValueError('safe_zone_start_tolerance must be greater than 0.0')
        if self.arm_sequence_service_timeout <= 0.0:
            raise ValueError('arm_sequence_service_timeout must be greater than 0.0')
        if self.arm_drop_service_timeout <= 0.0:
            raise ValueError('arm_drop_service_timeout must be greater than 0.0')
        if self.metal_backup_distance < 0.0:
            raise ValueError('metal_backup_distance must be greater than or equal to 0.0')
        if self.metal_backup_speed <= 0.0:
            raise ValueError('metal_backup_speed must be greater than 0.0')
        if self.coverage_boundary_margin < 0.0:
            raise ValueError('coverage_boundary_margin must be greater than or equal to 0.0')
        if self.front_obstacle_stop_distance <= 0.0:
            raise ValueError('front_obstacle_stop_distance must be greater than 0.0')
        if self.front_obstacle_angle <= 0.0:
            raise ValueError('front_obstacle_angle must be greater than 0.0')

        map_qos = QoSProfile(
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1)

        self.action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.map_sub = self.create_subscription(
            OccupancyGrid,
            self.map_topic,
            self.map_callback,
            map_qos)
        self.clicked_point_sub = self.create_subscription(
            PointStamped,
            self.clicked_corner_topic,
            self.clicked_point_callback,
            10)
        scan_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10)
        self.scan_sub = self.create_subscription(
            LaserScan,
            self.scan_topic,
            self.scan_callback,
            scan_qos)
        self.metal_event_sub = self.create_subscription(
            String,
            self.metal_events_topic,
            self.metal_event_callback,
            10)
        self.metal_state_sub = self.create_subscription(
            Bool,
            self.metal_state_topic,
            self.metal_state_callback,
            10)
        self.grid_pub = self.create_publisher(OccupancyGrid, self.grid_topic, 10)
        self.path_pub = self.create_publisher(Path, self.path_topic, 10)
        self.cmd_vel_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.grid_marker_pub = self.create_publisher(MarkerArray, self.grid_marker_topic, 10)
        self.priority_marker_pub = self.create_publisher(MarkerArray, self.priority_marker_topic, 10)
        self.mission_status_pub = self.create_publisher(String, self.mission_status_topic, 10)
        self.arm_sequence_client = self.create_client(Trigger, self.arm_sequence_service)
        self.arm_drop_client = self.create_client(Trigger, self.arm_drop_service)
        self.start_srv = self.create_service(Trigger, 'start_coverage', self.start_coverage_callback)
        self.stop_srv = self.create_service(Trigger, 'stop_coverage', self.stop_coverage_callback)

        self.source_map = None
        self.grid_msg = None
        self.grid_width = 0
        self.grid_height = 0
        self.cells_per_coverage = 1
        self.grid_min_u = 0.0
        self.grid_min_v = 0.0
        self.grid_cell_size_u = self.coverage_cell_size
        self.grid_cell_size_v = self.coverage_cell_size
        self.grid_axis_u = (1.0, 0.0)
        self.grid_axis_v = (0.0, 1.0)
        self.grid_yaw = 0.0
        self.clicked_corners = []
        self.manual_corners = self.parse_corner_points(self.corner_points)
        self.manual_safe_zone_point = self.parse_world_point(self.safe_zone_point)
        self.latest_scan = None
        self.obstacle_stop_active = False
        self.coverage_path = []
        self.return_path = []
        self.current_index = 0
        self.return_index = 0
        self.active_goal = False
        self.started = False
        self.goal_sent_time = None
        self.goal_handle = None
        self.goal_token = 0
        self.follower_state = 'rotate'
        self.coverage_state = 'coverage'
        self.return_resume_index = None
        self.mine_cells = set()
        self.metal_handling_state = 'idle'
        self.metal_sequence_future = None
        self.metal_sequence_start_time = None
        self.arm_drop_future = None
        self.arm_drop_start_time = None
        self.metal_backup_start_pose = None
        self.metal_event_cell = None
        self.metal_resume_index = None
        self.carried_mine_cell = None
        self.status_publish_count = 0
        self.status_publish_interval = max(1, int(round(self.control_frequency)))

        self.timer = self.create_timer(1.0 / self.control_frequency, self.timer_callback)
        self.get_logger().info(
            'Waiting for map on %s. corner_mode=%s. Use RViz Publish Point 4 times for clicked mode.'
            % (self.map_topic, self.corner_mode))

    def load_coverage_config(self):
        if not self.coverage_config_file:
            return

        try:
            with open(self.coverage_config_file, 'r', encoding='utf-8') as config_file:
                config = yaml.safe_load(config_file) or {}
        except OSError as exc:
            self.get_logger().warn(
                'Failed to read coverage_config_file "%s": %s'
                % (self.coverage_config_file, exc))
            return

        self.coverage_cell_size = float(config.get('coverage_cell_size', self.coverage_cell_size))
        self.path_strategy = config.get('path_strategy', self.path_strategy)
        self.stripe_axis = config.get('stripe_axis', self.stripe_axis)
        self.execution_mode = config.get('execution_mode', self.execution_mode)
        self.robot_base_frame = config.get('robot_base_frame', self.robot_base_frame)
        self.cmd_vel_topic = config.get('cmd_vel_topic', self.cmd_vel_topic)
        self.control_frequency = float(config.get('control_frequency', self.control_frequency))
        self.linear_speed = float(config.get('linear_speed', self.linear_speed))
        self.lookahead_distance = float(config.get('lookahead_distance', self.lookahead_distance))
        self.max_angular_speed = float(config.get('max_angular_speed', self.max_angular_speed))
        self.goal_tolerance = float(config.get('goal_tolerance', self.goal_tolerance))
        self.straight_goal_tolerance = float(
            config.get('straight_goal_tolerance', self.straight_goal_tolerance))
        self.corner_goal_tolerance = float(
            config.get('corner_goal_tolerance', self.corner_goal_tolerance))
        self.return_goal_tolerance = float(
            config.get('return_goal_tolerance', self.return_goal_tolerance))
        self.corner_angle_threshold = float(
            config.get('corner_angle_threshold', self.corner_angle_threshold))
        self.yaw_tolerance = float(config.get('yaw_tolerance', self.yaw_tolerance))
        self.rotate_to_path_yaw = float(config.get('rotate_to_path_yaw', self.rotate_to_path_yaw))
        self.enable_reverse_goal_recovery = bool(
            config.get('enable_reverse_goal_recovery', self.enable_reverse_goal_recovery))
        self.reverse_speed = float(config.get('reverse_speed', self.reverse_speed))
        self.reverse_goal_recovery_distance = float(
            config.get('reverse_goal_recovery_distance', self.reverse_goal_recovery_distance))
        self.path_interpolation_resolution = float(
            config.get('path_interpolation_resolution', self.path_interpolation_resolution))
        self.coverage_boundary_margin = float(
            config.get('coverage_boundary_margin', self.coverage_boundary_margin))
        self.corner_mode = config.get('corner_mode', self.corner_mode)
        self.clicked_corner_topic = config.get('clicked_corner_topic', self.clicked_corner_topic)
        self.require_free_cell_center = bool(
            config.get('require_free_cell_center', self.require_free_cell_center))
        self.safe_zone_path_index = int(
            config.get('safe_zone_path_index', self.safe_zone_path_index))
        self.safe_zone_point = config.get('safe_zone_point', self.safe_zone_point)
        self.start_direction_mode = config.get('start_direction_mode', self.start_direction_mode)
        self.require_start_in_safe_zone = bool(
            config.get('require_start_in_safe_zone', self.require_start_in_safe_zone))
        self.safe_zone_start_tolerance = float(
            config.get('safe_zone_start_tolerance', self.safe_zone_start_tolerance))
        self.reset_to_safe_zone_on_start = bool(
            config.get('reset_to_safe_zone_on_start', self.reset_to_safe_zone_on_start))
        self.return_to_safe_zone_on_complete = bool(
            config.get('return_to_safe_zone_on_complete', self.return_to_safe_zone_on_complete))
        self.return_to_safe_zone_on_mine = bool(
            config.get('return_to_safe_zone_on_mine', self.return_to_safe_zone_on_mine))
        self.safe_zone_drop_yaw_rad = float(
            config.get('safe_zone_drop_yaw_rad', self.safe_zone_drop_yaw_rad))
        self.simulated_mine_path_indices = self.parse_int_set(
            config.get('simulated_mine_path_indices', self.simulated_mine_path_indices))
        self.mine_detection_mode = config.get('mine_detection_mode', self.mine_detection_mode)
        self.metal_events_topic = config.get('metal_events_topic', self.metal_events_topic)
        self.metal_state_topic = config.get('metal_state_topic', self.metal_state_topic)
        self.metal_event_keywords = self.parse_string_list(
            config.get('metal_event_keywords', self.metal_event_keywords))
        self.arm_sequence_service = config.get('arm_sequence_service', self.arm_sequence_service)
        self.arm_sequence_service_timeout = float(
            config.get('arm_sequence_service_timeout', self.arm_sequence_service_timeout))
        self.arm_drop_service = config.get('arm_drop_service', self.arm_drop_service)
        self.arm_drop_service_timeout = float(
            config.get('arm_drop_service_timeout', self.arm_drop_service_timeout))
        self.metal_backup_distance = float(
            config.get('metal_backup_distance', self.metal_backup_distance))
        self.metal_backup_speed = float(config.get('metal_backup_speed', self.metal_backup_speed))
        self.continue_after_arm_failure = bool(
            config.get('continue_after_arm_failure', self.continue_after_arm_failure))
        self.mission_status_topic = config.get('mission_status_topic', self.mission_status_topic)
        self.scan_topic = config.get('scan_topic', self.scan_topic)
        self.enable_obstacle_stop = bool(
            config.get('enable_obstacle_stop', self.enable_obstacle_stop))
        self.front_obstacle_stop_distance = float(
            config.get('front_obstacle_stop_distance', self.front_obstacle_stop_distance))
        self.front_obstacle_angle = float(
            config.get('front_obstacle_angle', self.front_obstacle_angle))

        if 'corner_points' in config:
            self.corner_points = config['corner_points']
            self.manual_corners = self.parse_corner_points(self.corner_points)
        self.manual_safe_zone_point = self.parse_world_point(self.safe_zone_point)

        self.get_logger().info(
            'Loaded coverage config: %s' % self.coverage_config_file)

    def map_callback(self, msg):
        self.source_map = msg
        if self.coverage_path:
            return

        if self.corner_mode == 'clicked' and len(self.clicked_corners) < 4:
            self.get_logger().info(
                'Map received. Waiting for 4 clicked corners: %d/4'
                % len(self.clicked_corners))
            return

        self.rebuild_plan()

    def clicked_point_callback(self, msg):
        if self.corner_mode != 'clicked':
            return
        if len(self.clicked_corners) >= 4:
            self.get_logger().warn('Already have 4 corners. Restart node to select a new rectangle.')
            return

        self.clicked_corners.append((msg.point.x, msg.point.y))
        self.get_logger().info(
            'Clicked coverage corner %d/4: (%.3f, %.3f)'
            % (len(self.clicked_corners), msg.point.x, msg.point.y))

        if len(self.clicked_corners) == 4 and self.source_map is not None:
            self.rebuild_plan()

    def scan_callback(self, msg):
        self.latest_scan = msg

    def metal_event_callback(self, msg):
        if self.mine_detection_mode not in ('sensor', 'both'):
            return
        if not self.started or self.metal_handling_state != 'idle':
            return
        if self.coverage_state != 'coverage':
            return

        event_text = msg.data.strip()
        if not self.is_metal_detected_event(event_text):
            return

        self.start_metal_handling(event_text, self.current_index)

    def metal_state_callback(self, msg):
        if self.mine_detection_mode not in ('sensor', 'both'):
            return
        if not msg.data:
            return
        if not self.started or self.metal_handling_state != 'idle':
            return
        if self.coverage_state != 'coverage':
            return

        self.start_metal_handling('metal_state:true', self.current_index)

    def is_metal_detected_event(self, event_text):
        if event_text.startswith('METAL,'):
            tokens = [token.strip() for token in event_text.split(',')]
            if len(tokens) >= 5:
                return tokens[4] == '1'
            return False
        if not self.metal_event_keywords:
            return bool(event_text)
        lowered = event_text.lower()
        return any(keyword.lower() in lowered for keyword in self.metal_event_keywords)

    def start_metal_handling(self, event_text, resume_index):
        robot_pose = self.get_robot_pose()
        if robot_pose is None:
            self.publish_stop()
            self.started = False
            self.publish_mission_status('failed: cannot localize robot after metal event')
            return

        robot_x, robot_y, _ = robot_pose
        event_cell = self.world_to_grid_cell(robot_x, robot_y)
        if event_cell is None:
            if 0 <= self.current_index < len(self.coverage_path):
                event_cell = self.coverage_path[self.current_index]
            else:
                self.publish_stop()
                self.started = False
                self.publish_mission_status('failed: metal event outside coverage grid')
                return

        path_index = self.find_coverage_path_index(event_cell, self.current_index)
        if path_index is not None:
            resume_index = max(resume_index, path_index + 1)

        self.publish_stop()
        self.mark_cell(event_cell, CellState.VISITED)
        self.publish_grid()
        self.mine_cells.add(event_cell)
        self.metal_event_cell = event_cell
        self.metal_resume_index = resume_index
        self.metal_backup_start_pose = (robot_x, robot_y)
        self.metal_sequence_future = None
        self.metal_sequence_start_time = None
        self.metal_handling_state = 'backup' if self.metal_backup_distance > 0.0 else 'call_arm'
        self.publish_mission_status(
            'metal_detected: event="%s", cell=%s' % (event_text, str(event_cell)))

    def rebuild_plan(self):
        self.started = False
        self.active_goal = False
        self.current_index = 0
        self.return_index = 0
        self.coverage_path = []
        self.return_path = []
        self.follower_state = 'rotate'
        self.coverage_state = 'coverage'
        self.return_resume_index = None
        self.mine_cells = set()
        self.metal_handling_state = 'idle'
        self.metal_sequence_future = None
        self.metal_sequence_start_time = None
        self.metal_backup_start_pose = None
        self.metal_event_cell = None
        self.metal_resume_index = None
        self.carried_mine_cell = None

        self.build_grid_occupancy(self.source_map)
        self.coverage_path = self.make_coverage_path()
        self.apply_start_direction_policy()
        self.publish_grid()
        self.publish_path()
        self.publish_grid_markers()
        self.publish_priority_markers()

        self.get_logger().info(
            'Coverage grid ready: %d x %d cells, strategy=%s, stripe_axis=%s, %d goals'
            % (
                self.grid_width,
                self.grid_height,
                self.path_strategy,
                self.stripe_axis,
                len(self.coverage_path)))

        if self.start_automatically:
            self.started = True

    def apply_start_direction_policy(self):
        if len(self.coverage_path) < 2:
            return
        if self.start_direction_mode == 'path_order':
            return
        if self.manual_safe_zone_point is None:
            self.get_logger().warn(
                'start_direction_mode=%s requires safe_zone_point. '
                'Keeping generated path order.'
                % self.start_direction_mode)
            return

        if self.start_direction_mode == 'safe_zone_nearest_corner':
            self.apply_safe_zone_nearest_corner_policy()
            return

        if self.start_direction_mode != 'safe_zone_nearest_endpoint':
            self.get_logger().warn(
                'Unknown start_direction_mode "%s". Keeping generated path order.'
                % self.start_direction_mode)
            return

        safe_x, safe_y = self.manual_safe_zone_point
        first_x, first_y = self.path_cell_world(0)
        last_x, last_y = self.path_cell_world(len(self.coverage_path) - 1)
        first_distance = math.hypot(first_x - safe_x, first_y - safe_y)
        last_distance = math.hypot(last_x - safe_x, last_y - safe_y)

        if last_distance < first_distance:
            self.coverage_path.reverse()
            self.get_logger().info(
                'Coverage path reversed: end point is closer to safe_zone_point '
                '(%.2f m < %.2f m).'
                % (last_distance, first_distance))
        else:
            self.get_logger().info(
                'Coverage path kept: first point is closer to safe_zone_point '
                '(%.2f m <= %.2f m).'
                % (first_distance, last_distance))

    def apply_safe_zone_nearest_corner_policy(self):
        safe_x, safe_y = self.manual_safe_zone_point
        candidates = []

        for axis in ('x', 'y'):
            path = self.make_coverage_path(axis)
            if not path:
                continue
            candidates.append((axis, False, path, self.distance_from_cell_to_point(path[0], safe_x, safe_y)))
            reversed_path = list(reversed(path))
            candidates.append((
                axis,
                True,
                reversed_path,
                self.distance_from_cell_to_point(reversed_path[0], safe_x, safe_y)))

        if not candidates:
            self.get_logger().warn('No coverage path candidates were available.')
            return

        axis, reversed_order, path, distance = min(candidates, key=lambda candidate: candidate[3])
        self.stripe_axis = axis
        self.coverage_path = path
        self.get_logger().info(
            'Coverage path selected: stripe_axis=%s, reversed=%s, start distance to safe_zone_point=%.2f m.'
            % (axis, str(reversed_order).lower(), distance))

    def distance_from_cell_to_point(self, cell, world_x, world_y):
        cell_x, cell_y = self.grid_cell_world_center(cell[0], cell[1])
        return math.hypot(cell_x - world_x, cell_y - world_y)

    def start_coverage_callback(self, request, response):
        del request

        if self.grid_msg is None or not self.coverage_path:
            response.success = False
            response.message = 'Coverage grid is not ready. Check that /map is being published.'
            return response

        safe_index = self.get_safe_zone_index()
        if self.reset_to_safe_zone_on_start:
            self.current_index = safe_index

        if self.require_start_in_safe_zone:
            robot_pose = self.get_robot_pose()
            if robot_pose is None:
                response.success = False
                response.message = 'Cannot verify safe-zone start pose. Check TF map -> %s.' % (
                    self.robot_base_frame)
                return response

            robot_x, robot_y, _ = robot_pose
            safe_x, safe_y = self.path_cell_world(safe_index)
            distance = math.hypot(safe_x - robot_x, safe_y - robot_y)
            if distance > self.safe_zone_start_tolerance:
                response.success = False
                response.message = (
                    'Robot is %.2f m from safe zone %d. Move within %.2f m before starting.'
                    % (distance, safe_index + 1, self.safe_zone_start_tolerance))
                return response

        if self.current_index >= len(self.coverage_path):
            response.success = False
            response.message = 'Coverage path is already complete. Restart the node to plan again.'
            return response

        self.started = True
        if self.execution_mode == 'path_follower':
            self.active_goal = False
            self.goal_handle = None
            self.follower_state = 'rotate'
            self.coverage_state = 'coverage'
            self.return_resume_index = None
            if self.current_index >= len(self.coverage_path):
                self.current_index = 0
        response.success = True
        response.message = 'Coverage started.'
        self.get_logger().info(response.message)
        return response

    def get_safe_zone_index(self):
        return min(max(self.safe_zone_path_index - 1, 0), len(self.coverage_path) - 1)

    def stop_coverage_callback(self, request, response):
        del request

        self.started = False
        self.publish_stop()
        if self.active_goal and self.goal_handle is not None:
            self.goal_handle.cancel_goal_async()
            self.goal_token += 1

        self.active_goal = False
        self.goal_sent_time = None
        self.goal_handle = None

        response.success = True
        response.message = 'Coverage stopped.'
        self.get_logger().info(response.message)
        return response

    def timer_callback(self):
        self.status_publish_count += 1
        if self.grid_msg is not None and self.status_publish_count >= self.status_publish_interval:
            self.status_publish_count = 0
            self.publish_grid()
            self.publish_grid_markers()
            self.publish_priority_markers()

        if not self.started or not self.coverage_path:
            return

        if self.metal_handling_state != 'idle':
            self.process_metal_handling()
            return

        if self.coverage_state == 'align_after_mine':
            self.process_safe_zone_yaw_alignment()
            return

        if self.coverage_state == 'drop_after_mine':
            self.process_safe_zone_drop()
            return

        if self.execution_mode == 'path_follower':
            self.follow_coverage_path()
            return

        if self.active_goal:
            self.check_goal_timeout()
            return

        if self.current_index >= len(self.coverage_path):
            self.get_logger().info('Coverage complete: all planned cells were visited.')
            self.started = False
            return

        self.send_next_goal()

    def check_goal_timeout(self):
        if self.goal_sent_time is None or self.goal_timeout_sec <= 0.0:
            return

        elapsed = (self.get_clock().now() - self.goal_sent_time).nanoseconds / 1e9
        if elapsed <= self.goal_timeout_sec:
            return

        self.get_logger().warn('Goal timeout. Canceling and skipping current cell.')
        if self.goal_handle is not None:
            self.goal_handle.cancel_goal_async()
        self.mark_current_segment(CellState.VISITED)
        self.current_index += 1
        self.active_goal = False
        self.goal_sent_time = None
        self.goal_handle = None
        self.goal_token += 1

    def build_grid_occupancy(self, source_map):
        corners = self.get_active_corners()
        if corners:
            self.configure_grid_from_corners(corners)
        elif self.corner_mode in ('clicked', 'param'):
            self.get_logger().warn(
                'Coverage corners are not ready. corner_mode=%s requires exactly 4 corners.'
                % self.corner_mode)
            self.grid_msg = None
            return
        else:
            free_points = self.collect_free_world_points(source_map)
            if not free_points:
                self.get_logger().warn('No free cells were found in the map.')
                self.grid_msg = OccupancyGrid()
                return
            self.configure_rotated_grid(free_points)

        grid = OccupancyGrid()
        grid.header = source_map.header
        grid.info.resolution = self.coverage_cell_size
        grid.info.width = self.grid_width
        grid.info.height = self.grid_height
        grid_origin_x, grid_origin_y = self.grid_local_to_world(self.grid_min_u, self.grid_min_v)
        grid.info.origin.position.x = grid_origin_x
        grid.info.origin.position.y = grid_origin_y
        grid.info.origin.position.z = 0.0
        grid.info.origin.orientation.z = math.sin(self.grid_yaw * 0.5)
        grid.info.origin.orientation.w = math.cos(self.grid_yaw * 0.5)
        grid.data = [CellState.BLOCKED] * (self.grid_width * self.grid_height)

        for gy in range(self.grid_height):
            for gx in range(self.grid_width):
                wx, wy = self.grid_cell_world_center(gx, gy)
                if not self.require_free_cell_center or self.is_world_cell_free(source_map, wx, wy):
                    grid.data[self.grid_index(gx, gy)] = CellState.UNVISITED

        self.grid_msg = grid

    def get_active_corners(self):
        if self.corner_mode == 'clicked':
            return self.clicked_corners if len(self.clicked_corners) == 4 else []
        if self.corner_mode == 'param':
            return self.manual_corners if len(self.manual_corners) == 4 else []
        return []

    def parse_corner_points(self, values):
        if isinstance(values, str):
            if not values.strip():
                return []
            values = [
                float(value.strip())
                for value in values.split(',')
                if value.strip()
            ]
        if len(values) != 8:
            return []
        corners = []
        for index in range(0, 8, 2):
            corners.append((float(values[index]), float(values[index + 1])))
        return corners

    def parse_world_point(self, values):
        if isinstance(values, str):
            if not values.strip():
                return None
            values = [
                float(value.strip())
                for value in values.split(',')
                if value.strip()
            ]
        if values is None or len(values) != 2:
            return None
        return float(values[0]), float(values[1])

    def parse_int_set(self, values):
        if values is None:
            return set()
        if isinstance(values, set):
            return set(int(value) for value in values)
        if isinstance(values, (list, tuple)):
            return set(int(value) for value in values)
        if isinstance(values, str):
            if not values.strip():
                return set()
            return set(
                int(value.strip())
                for value in values.split(',')
                if value.strip())
        return set()

    def parse_string_list(self, values):
        if values is None:
            return []
        if isinstance(values, (list, tuple, set)):
            return [str(value).strip() for value in values if str(value).strip()]
        if isinstance(values, str):
            return [
                value.strip()
                for value in values.split(',')
                if value.strip()
            ]
        return []

    def configure_grid_from_corners(self, corners):
        ordered = self.order_corners(corners)
        longest_index = 0
        longest_length = -1.0

        for index in range(4):
            x0, y0 = ordered[index]
            x1, y1 = ordered[(index + 1) % 4]
            length = math.hypot(x1 - x0, y1 - y0)
            if length > longest_length:
                longest_length = length
                longest_index = index

        ux0, uy0 = ordered[longest_index]
        ux1, uy1 = ordered[(longest_index + 1) % 4]
        yaw = math.atan2(uy1 - uy0, ux1 - ux0)
        self.grid_yaw = yaw
        self.grid_axis_u = (math.cos(yaw), math.sin(yaw))
        self.grid_axis_v = (-math.sin(yaw), math.cos(yaw))

        projected = [self.world_to_grid_local(x, y) for x, y in corners]
        min_u = min(point[0] for point in projected)
        max_u = max(point[0] for point in projected)
        min_v = min(point[1] for point in projected)
        max_v = max(point[1] for point in projected)
        min_u, max_u, min_v, max_v = self.apply_boundary_margin(min_u, max_u, min_v, max_v)

        span_u = max(max_u - min_u, self.coverage_cell_size)
        span_v = max(max_v - min_v, self.coverage_cell_size)
        self.grid_width = max(1, int(math.ceil(span_u / self.coverage_cell_size)))
        self.grid_height = max(1, int(math.ceil(span_v / self.coverage_cell_size)))
        self.grid_cell_size_u = span_u / self.grid_width
        self.grid_cell_size_v = span_v / self.grid_height
        self.grid_min_u = min_u
        self.grid_min_v = min_v

    def order_corners(self, corners):
        center_x = sum(point[0] for point in corners) / len(corners)
        center_y = sum(point[1] for point in corners) / len(corners)
        return sorted(
            corners,
            key=lambda point: math.atan2(point[1] - center_y, point[0] - center_x))

    def collect_free_world_points(self, source_map):
        points = []
        for my in range(source_map.info.height):
            row_offset = my * source_map.info.width
            for mx in range(source_map.info.width):
                value = source_map.data[row_offset + mx]
                if value < 0 or value >= self.occupied_threshold:
                    continue
                points.append(self.source_cell_to_world(source_map, mx, my))
        return points

    def configure_rotated_grid(self, free_points):
        mean_x = sum(point[0] for point in free_points) / len(free_points)
        mean_y = sum(point[1] for point in free_points) / len(free_points)
        cov_xx = 0.0
        cov_xy = 0.0
        cov_yy = 0.0

        for x, y in free_points:
            dx = x - mean_x
            dy = y - mean_y
            cov_xx += dx * dx
            cov_xy += dx * dy
            cov_yy += dy * dy

        angle = 0.5 * math.atan2(2.0 * cov_xy, cov_xx - cov_yy)
        self.grid_yaw = angle
        self.grid_axis_u = (math.cos(angle), math.sin(angle))
        self.grid_axis_v = (-math.sin(angle), math.cos(angle))

        projected = [
            self.world_to_grid_local(x, y)
            for x, y in free_points
        ]
        min_u = min(point[0] for point in projected)
        max_u = max(point[0] for point in projected)
        min_v = min(point[1] for point in projected)
        max_v = max(point[1] for point in projected)
        min_u, max_u, min_v, max_v = self.apply_boundary_margin(min_u, max_u, min_v, max_v)

        span_u = max(max_u - min_u, self.coverage_cell_size)
        span_v = max(max_v - min_v, self.coverage_cell_size)
        self.grid_width = max(1, int(math.ceil(span_u / self.coverage_cell_size)))
        self.grid_height = max(1, int(math.ceil(span_v / self.coverage_cell_size)))
        self.grid_cell_size_u = span_u / self.grid_width
        self.grid_cell_size_v = span_v / self.grid_height
        self.grid_min_u = min_u
        self.grid_min_v = min_v

    def apply_boundary_margin(self, min_u, max_u, min_v, max_v):
        margin = self.coverage_boundary_margin
        if margin <= 0.0:
            return min_u, max_u, min_v, max_v

        span_u = max_u - min_u
        span_v = max_v - min_v
        max_margin_u = max(0.0, (span_u - self.coverage_cell_size) * 0.5)
        max_margin_v = max(0.0, (span_v - self.coverage_cell_size) * 0.5)
        margin_u = min(margin, max_margin_u)
        margin_v = min(margin, max_margin_v)

        if margin_u < margin or margin_v < margin:
            self.get_logger().warn(
                'coverage_boundary_margin was limited to fit the selected area: u=%.3f, v=%.3f'
                % (margin_u, margin_v))

        return (
            min_u + margin_u,
            max_u - margin_u,
            min_v + margin_v,
            max_v - margin_v)

    def is_coverage_cell_free(self, source_map, grid_x, grid_y, scale):
        start_x = grid_x * scale
        start_y = grid_y * scale
        end_x = min(start_x + scale, source_map.info.width)
        end_y = min(start_y + scale, source_map.info.height)

        sampled = 0
        for my in range(start_y, end_y):
            row_offset = my * source_map.info.width
            for mx in range(start_x, end_x):
                value = source_map.data[row_offset + mx]
                sampled += 1
                if value < 0 and self.unknown_is_blocked:
                    return False
                if value >= self.occupied_threshold:
                    return False

        return sampled > 0

    def is_world_cell_free(self, source_map, world_x, world_y):
        mx, my = self.world_to_source_cell(source_map, world_x, world_y)
        if mx < 0 or my < 0 or mx >= source_map.info.width or my >= source_map.info.height:
            return False

        value = source_map.data[my * source_map.info.width + mx]
        if value < 0 and self.unknown_is_blocked:
            return False
        return 0 <= value < self.occupied_threshold

    def source_cell_to_world(self, source_map, mx, my):
        local_x = (mx + 0.5) * source_map.info.resolution
        local_y = (my + 0.5) * source_map.info.resolution
        yaw = self.get_yaw_from_pose(source_map.info.origin)
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)
        return (
            source_map.info.origin.position.x + local_x * cos_yaw - local_y * sin_yaw,
            source_map.info.origin.position.y + local_x * sin_yaw + local_y * cos_yaw)

    def world_to_source_cell(self, source_map, world_x, world_y):
        yaw = self.get_yaw_from_pose(source_map.info.origin)
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)
        dx = world_x - source_map.info.origin.position.x
        dy = world_y - source_map.info.origin.position.y
        local_x = dx * cos_yaw + dy * sin_yaw
        local_y = -dx * sin_yaw + dy * cos_yaw
        return (
            int(math.floor(local_x / source_map.info.resolution)),
            int(math.floor(local_y / source_map.info.resolution)))

    def get_yaw_from_pose(self, pose):
        q = pose.orientation
        return self.get_yaw_from_quaternion(q)

    def get_yaw_from_quaternion(self, q):
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    def make_boustrophedon_path(self, stripe_axis=None):
        axis = stripe_axis or self.stripe_axis
        path = []

        if axis == 'y':
            for gx in range(self.grid_width):
                y_range = (
                    range(self.grid_height)
                    if gx % 2 == 0
                    else range(self.grid_height - 1, -1, -1)
                )
                for gy in y_range:
                    if self.grid_msg.data[self.grid_index(gx, gy)] == CellState.UNVISITED:
                        path.append((gx, gy))
            return path

        for gy in range(self.grid_height):
            x_range = (
                range(self.grid_width)
                if gy % 2 == 0
                else range(self.grid_width - 1, -1, -1)
            )
            for gx in x_range:
                if self.grid_msg.data[self.grid_index(gx, gy)] == CellState.UNVISITED:
                    path.append((gx, gy))
        return path

    def make_coverage_path(self, stripe_axis=None):
        if self.path_strategy == 'cell_centers':
            return self.make_boustrophedon_path(stripe_axis)

        if self.path_strategy != 'stripe_endpoints':
            self.get_logger().warn(
                'Unknown path_strategy "%s". Falling back to stripe_endpoints.'
                % self.path_strategy)

        return self.make_stripe_endpoint_path(stripe_axis)

    def make_stripe_endpoint_path(self, stripe_axis=None):
        axis = stripe_axis or self.stripe_axis
        if axis == 'y':
            return self.make_vertical_stripe_endpoint_path()

        path = []
        left_to_right = True

        for gy in range(self.grid_height):
            free_xs = [
                gx for gx in range(self.grid_width)
                if self.grid_msg.data[self.grid_index(gx, gy)] == CellState.UNVISITED
            ]
            if not free_xs:
                continue

            left = min(free_xs)
            right = max(free_xs)
            row_start = (left, gy) if left_to_right else (right, gy)
            row_end = (right, gy) if left_to_right else (left, gy)

            if not path or path[-1] != row_start:
                path.append(row_start)
            if row_end != row_start:
                path.append(row_end)

            left_to_right = not left_to_right

        return path

    def make_vertical_stripe_endpoint_path(self):
        path = []
        bottom_to_top = True

        for gx in range(self.grid_width):
            free_ys = [
                gy for gy in range(self.grid_height)
                if self.grid_msg.data[self.grid_index(gx, gy)] == CellState.UNVISITED
            ]
            if not free_ys:
                continue

            bottom = min(free_ys)
            top = max(free_ys)
            col_start = (gx, bottom) if bottom_to_top else (gx, top)
            col_end = (gx, top) if bottom_to_top else (gx, bottom)

            if not path or path[-1] != col_start:
                path.append(col_start)
            if col_end != col_start:
                path.append(col_end)

            bottom_to_top = not bottom_to_top

        return path

    def follow_coverage_path(self):
        active_path = self.get_active_path()
        active_index = self.get_active_path_index()
        if active_index >= len(active_path):
            self.start_return_to_safe_zone('complete')
            return

        robot_pose = self.get_robot_pose()
        if robot_pose is None:
            self.publish_stop()
            return

        robot_x, robot_y, robot_yaw = robot_pose
        target_x, target_y = self.path_cell_world(active_index)
        distance_to_target = math.hypot(target_x - robot_x, target_y - robot_y)
        active_goal_tolerance = self.get_goal_tolerance(active_index)

        if distance_to_target <= active_goal_tolerance:
            self.handle_target_reached()
            return

        desired_yaw = self.get_segment_heading(robot_x, robot_y)
        yaw_error = self.normalize_angle(desired_yaw - robot_yaw)

        if self.follower_state == 'rotate':
            if abs(yaw_error) > self.yaw_tolerance:
                self.publish_rotate(yaw_error)
                return
            self.publish_stop()
            self.follower_state = 'drive'
            return

        if abs(yaw_error) > self.rotate_to_path_yaw:
            self.publish_stop()
            self.follower_state = 'rotate'
            return

        target_x_local, target_y_local = self.world_point_to_robot(
            target_x,
            target_y,
            robot_x,
            robot_y,
            robot_yaw)
        if self.should_reverse_to_target(
            target_x_local,
            distance_to_target,
            active_goal_tolerance):
            self.publish_reverse_to_target(target_x_local, target_y_local)
            return

        if self.is_front_obstacle_close():
            self.publish_stop()
            return

        lookahead_x, lookahead_y = self.get_lookahead_point(robot_x, robot_y)
        x_local, y_local = self.world_point_to_robot(
            lookahead_x,
            lookahead_y,
            robot_x,
            robot_y,
            robot_yaw)

        if x_local <= 0.0 and abs(yaw_error) > self.yaw_tolerance:
            self.publish_stop()
            self.follower_state = 'rotate'
            return

        lookahead = max(math.hypot(x_local, y_local), 0.05)
        curvature = 2.0 * y_local / (lookahead * lookahead)
        angular = self.clamp(
            self.linear_speed * curvature,
            -self.max_angular_speed,
            self.max_angular_speed)

        cmd = Twist()
        cmd.linear.x = self.linear_speed
        cmd.angular.z = angular
        self.cmd_vel_pub.publish(cmd)

    def process_metal_handling(self):
        if self.metal_handling_state == 'backup':
            self.process_metal_backup()
            return
        if self.metal_handling_state == 'call_arm':
            self.call_arm_sequence()
            return
        if self.metal_handling_state == 'wait_arm':
            self.wait_for_arm_sequence()
            return

        self.publish_stop()

    def process_metal_backup(self):
        robot_pose = self.get_robot_pose()
        if robot_pose is None or self.metal_backup_start_pose is None:
            self.publish_stop()
            self.fail_metal_handling('failed: cannot localize robot during metal backup')
            return

        robot_x, robot_y, _ = robot_pose
        start_x, start_y = self.metal_backup_start_pose
        backed_distance = math.hypot(robot_x - start_x, robot_y - start_y)
        if backed_distance >= self.metal_backup_distance:
            self.publish_stop()
            self.metal_handling_state = 'call_arm'
            self.publish_mission_status('backup_complete: %.3f m' % backed_distance)
            return

        cmd = Twist()
        cmd.linear.x = -self.metal_backup_speed
        self.cmd_vel_pub.publish(cmd)

    def call_arm_sequence(self):
        self.publish_stop()
        if not self.arm_sequence_client.service_is_ready():
            if self.metal_sequence_start_time is None:
                self.metal_sequence_start_time = self.get_clock().now()
            elapsed = self.elapsed_since(self.metal_sequence_start_time)
            if elapsed > self.arm_sequence_service_timeout:
                self.fail_metal_handling(
                    'failed: arm sequence service unavailable: %s'
                    % self.arm_sequence_service)
            return

        self.metal_sequence_future = self.arm_sequence_client.call_async(Trigger.Request())
        self.metal_sequence_start_time = self.get_clock().now()
        self.metal_handling_state = 'wait_arm'
        self.publish_mission_status('arm_sequence_started')

    def wait_for_arm_sequence(self):
        self.publish_stop()
        if self.metal_sequence_future is None:
            self.metal_handling_state = 'call_arm'
            return

        if self.elapsed_since(self.metal_sequence_start_time) > self.arm_sequence_service_timeout:
            self.fail_metal_handling('failed: arm sequence timeout')
            return

        if not self.metal_sequence_future.done():
            return

        try:
            result = self.metal_sequence_future.result()
        except Exception as exc:
            self.fail_metal_handling('failed: arm sequence exception: %s' % exc)
            return

        if result.success:
            self.publish_mission_status('arm_sequence_succeeded: %s' % result.message)
            self.finish_metal_handling_success()
            return

        self.fail_metal_handling('failed: arm sequence failed: %s' % result.message)

    def finish_metal_handling_success(self):
        event_cell = self.metal_event_cell
        resume_index = self.metal_resume_index
        self.reset_metal_handling()
        if event_cell is None:
            self.publish_stop()
            self.started = False
            self.publish_mission_status('failed: missing metal event cell')
            return
        if not self.return_to_safe_zone_on_mine:
            if resume_index is None:
                resume_index = self.current_index
            if resume_index >= len(self.coverage_path):
                self.start_return_to_safe_zone('complete')
                return
            self.coverage_state = 'coverage'
            self.return_resume_index = None
            self.return_path = []
            self.return_index = 0
            self.current_index = resume_index
            self.follower_state = 'rotate'
            self.publish_mission_status(
                'arm_sequence_complete: resuming coverage at grid %d'
                % (self.current_index + 1))
            return
        self.carried_mine_cell = event_cell
        self.start_return_to_safe_zone('mine', resume_index=resume_index, start_cell=event_cell)

    def fail_metal_handling(self, message):
        self.publish_stop()
        self.publish_mission_status(message)

        if self.continue_after_arm_failure and self.metal_event_cell is not None:
            event_cell = self.metal_event_cell
            resume_index = self.metal_resume_index
            self.reset_metal_handling()
            self.start_return_to_safe_zone('mine', resume_index=resume_index, start_cell=event_cell)
            return

        self.started = False
        self.reset_metal_handling()

    def reset_metal_handling(self):
        self.metal_handling_state = 'idle'
        self.metal_sequence_future = None
        self.metal_sequence_start_time = None
        self.metal_backup_start_pose = None
        self.metal_event_cell = None
        self.metal_resume_index = None

    def elapsed_since(self, start_time):
        if start_time is None:
            return 0.0
        return (self.get_clock().now() - start_time).nanoseconds / 1e9

    def publish_mission_status(self, text):
        msg = String()
        msg.data = text
        self.mission_status_pub.publish(msg)
        self.get_logger().info(text)

    def get_active_path(self):
        if self.coverage_state == 'coverage':
            return self.coverage_path
        return self.return_path

    def get_active_path_index(self):
        if self.coverage_state == 'coverage':
            return self.current_index
        return self.return_index

    def handle_target_reached(self):
        self.publish_stop()

        if self.coverage_state != 'coverage' and self.return_index < len(self.return_path) - 1:
            self.return_index += 1
            self.follower_state = 'rotate'
            return

        if self.coverage_state == 'return_after_mine':
            resume_index = self.return_resume_index
            if resume_index is None or resume_index >= len(self.coverage_path):
                self.return_resume_index = None
                self.return_path = []
                self.return_index = 0
                self.start_return_to_safe_zone('complete')
                return
            self.coverage_state = 'align_after_mine'
            self.publish_mission_status(
                'safe_zone_reached: aligning yaw to %.3f rad before arm drop'
                % self.safe_zone_drop_yaw_rad)
            return

        if self.coverage_state == 'resume_after_mine':
            resume_index = self.return_resume_index
            self.coverage_state = 'coverage'
            self.return_resume_index = None
            self.return_path = []
            self.return_index = 0
            self.current_index = resume_index if resume_index is not None else self.current_index
            self.follower_state = 'rotate'
            self.get_logger().info(
                'Reached next unvisited grid %d via A*. Resuming coverage.'
                % (self.current_index + 1))
            return

        if self.coverage_state == 'return_complete':
            self.started = False
            self.follower_state = 'rotate'
            self.return_path = []
            self.return_index = 0
            self.get_logger().info('Coverage complete: returned to safe zone.')
            return

        reached_index = self.current_index
        self.mark_current_segment(CellState.VISITED)
        self.current_index += 1
        self.follower_state = 'rotate'

        if self.should_simulate_mine_at_index(reached_index):
            self.get_logger().warn(
                'Simulated mine detected at coverage grid %d. Returning to safe zone.'
                % (reached_index + 1))
            self.start_metal_handling('simulated_mine', self.current_index)
            return

        if self.current_index >= len(self.coverage_path):
            self.start_return_to_safe_zone('complete')

    def process_safe_zone_drop(self):
        self.publish_stop()
        if self.arm_drop_future is None:
            self.call_arm_drop_sequence()
            return

        self.wait_for_arm_drop_sequence()

    def process_safe_zone_yaw_alignment(self):
        robot_pose = self.get_robot_pose()
        if robot_pose is None:
            self.publish_stop()
            return

        _, _, robot_yaw = robot_pose
        yaw_error = self.normalize_angle(self.safe_zone_drop_yaw_rad - robot_yaw)
        if abs(yaw_error) > self.yaw_tolerance:
            self.publish_rotate(yaw_error)
            return

        self.publish_stop()
        self.coverage_state = 'drop_after_mine'
        self.arm_drop_future = None
        self.arm_drop_start_time = None
        self.publish_mission_status('safe_zone_yaw_aligned: starting arm drop sequence')

    def call_arm_drop_sequence(self):
        if not self.arm_drop_client.service_is_ready():
            if self.arm_drop_start_time is None:
                self.arm_drop_start_time = self.get_clock().now()
            elapsed = self.elapsed_since(self.arm_drop_start_time)
            if elapsed > self.arm_drop_service_timeout:
                self.publish_stop()
                self.started = False
                self.publish_mission_status(
                    'failed: arm drop service unavailable: %s' % self.arm_drop_service)
            return

        self.arm_drop_future = self.arm_drop_client.call_async(Trigger.Request())
        self.arm_drop_start_time = self.get_clock().now()
        self.publish_mission_status('arm_drop_started')

    def wait_for_arm_drop_sequence(self):
        if self.elapsed_since(self.arm_drop_start_time) > self.arm_drop_service_timeout:
            self.publish_stop()
            self.started = False
            self.publish_mission_status('failed: arm drop sequence timeout')
            return

        if not self.arm_drop_future.done():
            return

        try:
            result = self.arm_drop_future.result()
        except Exception as exc:
            self.publish_stop()
            self.started = False
            self.publish_mission_status('failed: arm drop sequence exception: %s' % exc)
            return

        if not result.success:
            self.publish_stop()
            self.started = False
            self.publish_mission_status('failed: arm drop sequence failed: %s' % result.message)
            return

        resume_index = self.return_resume_index
        self.arm_drop_future = None
        self.arm_drop_start_time = None
        self.clear_carried_mine_cell()
        self.publish_mission_status('arm_drop_succeeded: %s' % result.message)
        if resume_index is None or resume_index >= len(self.coverage_path):
            self.return_resume_index = None
            self.return_path = []
            self.return_index = 0
            self.start_return_to_safe_zone('complete')
            return
        self.start_resume_from_safe_zone(resume_index)

    def clear_carried_mine_cell(self):
        if self.carried_mine_cell is None:
            return

        cleared_cell = self.carried_mine_cell
        self.mine_cells.discard(cleared_cell)
        self.mark_cell(cleared_cell, CellState.VISITED)
        self.carried_mine_cell = None
        self.publish_grid()
        self.publish_mission_status('mine_cell_cleared: %s is now traversable' % (str(cleared_cell),))

    def should_simulate_mine_at_index(self, path_index):
        if self.mine_detection_mode not in ('simulated', 'both'):
            return False
        if not self.return_to_safe_zone_on_mine:
            return False
        return (path_index + 1) in self.simulated_mine_path_indices

    def start_return_to_safe_zone(self, reason, resume_index=None, start_cell=None):
        if reason == 'complete' and not self.return_to_safe_zone_on_complete:
            self.publish_stop()
            self.started = False
            self.get_logger().info('Coverage complete: path follower reached the end.')
            return

        safe_index = self.get_safe_zone_index()
        safe_cell = self.coverage_path[safe_index]
        if start_cell is None:
            if 0 <= self.current_index < len(self.coverage_path):
                start_cell = self.coverage_path[self.current_index]
            elif self.coverage_path:
                start_cell = self.coverage_path[-1]
            else:
                self.publish_stop()
                self.started = False
                return

        return_path = self.plan_grid_path_astar(start_cell, safe_cell)
        if not return_path:
            self.publish_stop()
            self.started = False
            self.get_logger().error(
                'Failed to plan safe-zone return path from %s to %s.'
                % (str(start_cell), str(safe_cell)))
            return

        self.return_path = return_path
        self.return_index = 0
        self.follower_state = 'rotate'

        if reason == 'mine':
            self.coverage_state = 'return_after_mine'
            self.return_resume_index = resume_index
            self.get_logger().info(
                'Returning to safe zone via A*: %d grid points.'
                % len(self.return_path))
            return

        self.coverage_state = 'return_complete'
        self.return_resume_index = None
        self.get_logger().info(
            'Coverage complete. Returning to safe zone via A*: %d grid points.'
            % len(self.return_path))

    def start_resume_from_safe_zone(self, resume_index):
        safe_cell = self.coverage_path[self.get_safe_zone_index()]
        target_cell = self.coverage_path[resume_index]
        resume_path = self.plan_grid_path_astar(safe_cell, target_cell)
        if not resume_path:
            self.publish_stop()
            self.started = False
            self.return_path = []
            self.return_index = 0
            self.return_resume_index = None
            self.get_logger().error(
                'Failed to plan resume path from safe zone %s to next grid %d %s.'
                % (str(safe_cell), resume_index + 1, str(target_cell)))
            return

        self.return_path = resume_path
        self.return_index = 0
        self.coverage_state = 'resume_after_mine'
        self.return_resume_index = resume_index
        self.follower_state = 'rotate'
        self.get_logger().info(
            'Returned to safe zone. Moving to next unvisited grid %d via A*: %d grid points.'
            % (resume_index + 1, len(self.return_path)))

    def plan_grid_path_astar(self, start, goal):
        start = (int(start[0]), int(start[1]))
        goal = (int(goal[0]), int(goal[1]))
        if not self.is_grid_cell_traversable(start, start, goal):
            return []
        if not self.is_grid_cell_traversable(goal, start, goal):
            return []

        open_heap = []
        heapq.heappush(open_heap, (self.grid_manhattan(start, goal), 0, start))
        came_from = {}
        cost_so_far = {start: 0}

        while open_heap:
            _, cost, current = heapq.heappop(open_heap)
            if current == goal:
                return self.reconstruct_grid_path(came_from, current)
            if cost > cost_so_far[current]:
                continue

            for neighbor in self.get_grid_neighbors(current):
                if not self.is_grid_cell_traversable(neighbor, start, goal):
                    continue
                new_cost = cost_so_far[current] + 1
                if neighbor not in cost_so_far or new_cost < cost_so_far[neighbor]:
                    cost_so_far[neighbor] = new_cost
                    priority = new_cost + self.grid_manhattan(neighbor, goal)
                    heapq.heappush(open_heap, (priority, new_cost, neighbor))
                    came_from[neighbor] = current

        return []

    def reconstruct_grid_path(self, came_from, current):
        path = [current]
        while current in came_from:
            current = came_from[current]
            path.append(current)
        path.reverse()
        return path

    def get_grid_neighbors(self, cell):
        gx, gy = cell
        candidates = [
            (gx + 1, gy),
            (gx - 1, gy),
            (gx, gy + 1),
            (gx, gy - 1),
        ]
        return [
            candidate
            for candidate in candidates
            if 0 <= candidate[0] < self.grid_width and 0 <= candidate[1] < self.grid_height
        ]

    def is_grid_cell_traversable(self, cell, start, goal):
        if cell != start and self.is_mine_cell(cell):
            return False
        gx, gy = cell
        if gx < 0 or gy < 0 or gx >= self.grid_width or gy >= self.grid_height:
            return False
        if cell == start or cell == goal:
            return True
        return self.grid_msg.data[self.grid_index(gx, gy)] == CellState.VISITED

    def is_mine_cell(self, cell):
        return cell in self.mine_cells

    def grid_manhattan(self, first, second):
        return abs(first[0] - second[0]) + abs(first[1] - second[1])

    def should_reverse_to_target(self, target_x_local, distance_to_target, active_goal_tolerance):
        if not self.enable_reverse_goal_recovery:
            return False
        if target_x_local >= -active_goal_tolerance:
            return False
        return distance_to_target <= self.reverse_goal_recovery_distance

    def publish_reverse_to_target(self, x_local, y_local):
        lookahead = max(math.hypot(x_local, y_local), 0.05)
        curvature = 2.0 * y_local / (lookahead * lookahead)
        angular = self.clamp(
            -self.reverse_speed * curvature,
            -self.max_angular_speed,
            self.max_angular_speed)

        cmd = Twist()
        cmd.linear.x = -self.reverse_speed
        cmd.angular.z = angular
        self.cmd_vel_pub.publish(cmd)

    def get_goal_tolerance(self, path_index):
        if self.coverage_state != 'coverage':
            return self.return_goal_tolerance
        if self.is_corner_path_index(path_index):
            return self.corner_goal_tolerance
        return self.straight_goal_tolerance

    def is_corner_path_index(self, path_index):
        if path_index <= 0 or path_index >= len(self.coverage_path) - 1:
            return False

        prev_x, prev_y = self.path_cell_world(path_index - 1)
        curr_x, curr_y = self.path_cell_world(path_index)
        next_x, next_y = self.path_cell_world(path_index + 1)

        in_heading = math.atan2(curr_y - prev_y, curr_x - prev_x)
        out_heading = math.atan2(next_y - curr_y, next_x - curr_x)
        if (
            math.hypot(curr_x - prev_x, curr_y - prev_y) <= 1e-6
            or math.hypot(next_x - curr_x, next_y - curr_y) <= 1e-6
        ):
            return False

        heading_change = abs(self.normalize_angle(out_heading - in_heading))
        return heading_change >= self.corner_angle_threshold

    def is_front_obstacle_close(self):
        if not self.enable_obstacle_stop or self.latest_scan is None:
            return False

        scan = self.latest_scan
        half_angle = self.front_obstacle_angle * 0.5
        closest = math.inf
        angle = scan.angle_min

        for distance in scan.ranges:
            if -half_angle <= angle <= half_angle:
                if (
                    math.isfinite(distance)
                    and distance >= scan.range_min
                    and distance <= scan.range_max
                ):
                    closest = min(closest, distance)
            angle += scan.angle_increment

        is_blocked = closest < self.front_obstacle_stop_distance
        if is_blocked and not self.obstacle_stop_active:
            self.get_logger().warn(
                'Front obstacle stop: closest=%.3f m, threshold=%.3f m'
                % (closest, self.front_obstacle_stop_distance))
        elif not is_blocked and self.obstacle_stop_active:
            self.get_logger().info('Front obstacle cleared. Resuming coverage path follower.')

        self.obstacle_stop_active = is_blocked
        return is_blocked

    def get_robot_pose(self):
        try:
            transform = self.tf_buffer.lookup_transform(
                self.goal_frame_id,
                self.robot_base_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=0.05))
        except TransformException as exc:
            self.get_logger().warn('Cannot get robot pose: %s' % exc)
            return None

        translation = transform.transform.translation
        yaw = self.get_yaw_from_quaternion(transform.transform.rotation)
        return translation.x, translation.y, yaw

    def get_segment_heading(self, robot_x, robot_y):
        active_index = self.get_active_path_index()
        if self.coverage_state != 'coverage' or active_index == 0:
            target_x, target_y = self.path_cell_world(active_index)
            return math.atan2(target_y - robot_y, target_x - robot_x)

        start_x, start_y = self.path_cell_world(active_index - 1)
        end_x, end_y = self.path_cell_world(active_index)
        return math.atan2(end_y - start_y, end_x - start_x)

    def get_lookahead_point(self, robot_x, robot_y):
        active_index = self.get_active_path_index()
        if self.coverage_state != 'coverage' or active_index == 0:
            return self.path_cell_world(active_index)

        start_x, start_y = self.path_cell_world(active_index - 1)
        end_x, end_y = self.path_cell_world(active_index)
        seg_x = end_x - start_x
        seg_y = end_y - start_y
        seg_len_sq = seg_x * seg_x + seg_y * seg_y

        if seg_len_sq <= 1e-9:
            return end_x, end_y

        projection = (
            (robot_x - start_x) * seg_x + (robot_y - start_y) * seg_y
        ) / seg_len_sq
        projection = self.clamp(projection, 0.0, 1.0)

        segment_length = math.sqrt(seg_len_sq)
        lookahead_ratio = self.lookahead_distance / segment_length
        target_ratio = self.clamp(projection + lookahead_ratio, 0.0, 1.0)

        return (
            start_x + seg_x * target_ratio,
            start_y + seg_y * target_ratio)

    def path_cell_world(self, index):
        path = self.get_active_path()
        gx, gy = path[index]
        return self.grid_cell_world_center(gx, gy)

    def world_point_to_robot(self, world_x, world_y, robot_x, robot_y, robot_yaw):
        dx = world_x - robot_x
        dy = world_y - robot_y
        cos_yaw = math.cos(robot_yaw)
        sin_yaw = math.sin(robot_yaw)
        return (
            cos_yaw * dx + sin_yaw * dy,
            -sin_yaw * dx + cos_yaw * dy)

    def publish_rotate(self, yaw_error):
        cmd = Twist()
        cmd.angular.z = self.clamp(
            1.5 * yaw_error,
            -self.max_angular_speed,
            self.max_angular_speed)
        self.cmd_vel_pub.publish(cmd)

    def publish_stop(self):
        self.cmd_vel_pub.publish(Twist())

    def clamp(self, value, lower, upper):
        return min(max(value, lower), upper)

    def send_next_goal(self):
        gx, gy = self.coverage_path[self.current_index]
        pose = self.grid_cell_to_pose(gx, gy, self.get_goal_yaw(self.current_index))
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose

        if not self.action_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().warn('Waiting for navigate_to_pose action server...')
            return

        self.active_goal = True
        self.goal_sent_time = self.get_clock().now()
        self.goal_token += 1
        token = self.goal_token
        future = self.action_client.send_goal_async(goal_msg)
        future.add_done_callback(lambda done: self.goal_response_callback(done, token))

        self.get_logger().info(
            'Sending coverage goal %d/%d: grid=(%d, %d), map=(%.2f, %.2f)'
            % (
                self.current_index + 1,
                len(self.coverage_path),
                gx,
                gy,
                pose.pose.position.x,
                pose.pose.position.y))

    def goal_response_callback(self, future, token):
        if token != self.goal_token:
            return

        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Coverage goal rejected. Marking cell visited and continuing.')
            self.mark_current_segment(CellState.VISITED)
            self.current_index += 1
            self.active_goal = False
            return

        self.goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(lambda done: self.goal_result_callback(done, token))

    def goal_result_callback(self, future, token):
        if token != self.goal_token:
            return

        result = future.result()
        if result.status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Coverage goal reached.')
        else:
            self.get_logger().warn('Coverage goal finished with status %d.' % result.status)

        self.mark_current_segment(CellState.VISITED)
        self.current_index += 1
        self.active_goal = False
        self.goal_sent_time = None
        self.goal_handle = None

    def mark_current_cell(self, state):
        if self.current_index >= len(self.coverage_path):
            return
        gx, gy = self.coverage_path[self.current_index]
        self.grid_msg.data[self.grid_index(gx, gy)] = state
        self.publish_grid()

    def mark_current_segment(self, state):
        if self.current_index >= len(self.coverage_path):
            return

        if self.current_index == 0 or self.path_strategy == 'cell_centers':
            self.mark_cell(self.coverage_path[self.current_index], state)
            self.publish_grid()
            return

        self.mark_line(
            self.coverage_path[self.current_index - 1],
            self.coverage_path[self.current_index],
            state)
        self.publish_grid()

    def mark_cell(self, cell, state):
        gx, gy = cell
        if gx < 0 or gy < 0 or gx >= self.grid_width or gy >= self.grid_height:
            return
        self.grid_msg.data[self.grid_index(gx, gy)] = state

    def mark_line(self, start, end, state):
        x0, y0 = start
        x1, y1 = end
        dx = x1 - x0
        dy = y1 - y0
        steps = max(abs(dx), abs(dy), 1)

        for step in range(steps + 1):
            ratio = step / steps
            gx = int(round(x0 + dx * ratio))
            gy = int(round(y0 + dy * ratio))
            self.mark_cell((gx, gy), state)

    def get_goal_yaw(self, index):
        if not self.face_next_goal or not self.coverage_path:
            return 0.0

        if index + 1 < len(self.coverage_path):
            from_cell = self.coverage_path[index]
            to_cell = self.coverage_path[index + 1]
        elif index > 0:
            from_cell = self.coverage_path[index - 1]
            to_cell = self.coverage_path[index]
        else:
            return 0.0

        from_x, from_y = self.grid_cell_world_center(from_cell[0], from_cell[1])
        to_x, to_y = self.grid_cell_world_center(to_cell[0], to_cell[1])
        dx = to_x - from_x
        dy = to_y - from_y
        if dx == 0 and dy == 0:
            return 0.0
        return math.atan2(dy, dx)

    def grid_cell_to_pose(self, gx, gy, yaw=0.0):
        pose = PoseStamped()
        pose.header.frame_id = self.goal_frame_id
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x, pose.pose.position.y = self.grid_cell_world_center(gx, gy)
        pose.pose.orientation.z = math.sin(yaw * 0.5)
        pose.pose.orientation.w = math.cos(yaw * 0.5)
        return pose

    def grid_cell_world_center(self, gx, gy):
        local_u = self.grid_min_u + (gx + 0.5) * self.grid_cell_size_u
        local_v = self.grid_min_v + (gy + 0.5) * self.grid_cell_size_v
        return self.grid_local_to_world(local_u, local_v)

    def grid_corner_world(self, gx, gy):
        local_u = self.grid_min_u + gx * self.grid_cell_size_u
        local_v = self.grid_min_v + gy * self.grid_cell_size_v
        return self.grid_local_to_world(local_u, local_v)

    def world_to_grid_local(self, world_x, world_y):
        return (
            world_x * self.grid_axis_u[0] + world_y * self.grid_axis_u[1],
            world_x * self.grid_axis_v[0] + world_y * self.grid_axis_v[1])

    def world_to_grid_cell(self, world_x, world_y):
        local_u, local_v = self.world_to_grid_local(world_x, world_y)
        gx = int(math.floor((local_u - self.grid_min_u) / self.grid_cell_size_u))
        gy = int(math.floor((local_v - self.grid_min_v) / self.grid_cell_size_v))
        if gx < 0 or gy < 0 or gx >= self.grid_width or gy >= self.grid_height:
            return None
        return gx, gy

    def grid_local_to_world(self, local_u, local_v):
        return (
            local_u * self.grid_axis_u[0] + local_v * self.grid_axis_v[0],
            local_u * self.grid_axis_u[1] + local_v * self.grid_axis_v[1])

    def find_coverage_path_index(self, cell, start_index=0):
        for index in range(max(start_index, 0), len(self.coverage_path)):
            if self.coverage_path[index] == cell:
                return index
        for index in range(0, min(start_index, len(self.coverage_path))):
            if self.coverage_path[index] == cell:
                return index
        return None

    def grid_index(self, gx, gy):
        return gy * self.grid_width + gx

    def publish_grid(self):
        if self.grid_msg is None:
            return
        self.grid_msg.header.stamp = self.get_clock().now().to_msg()
        self.grid_pub.publish(self.grid_msg)

    def publish_path(self):
        path = Path()
        path.header.frame_id = self.goal_frame_id
        path.header.stamp = self.get_clock().now().to_msg()
        path.poses = self.build_interpolated_path_poses()
        self.path_pub.publish(path)

    def build_interpolated_path_poses(self):
        if not self.coverage_path:
            return []

        poses = []
        for index, (gx, gy) in enumerate(self.coverage_path):
            if index == 0:
                poses.append(self.grid_cell_to_pose(gx, gy, self.get_goal_yaw(index)))
                continue

            start_x, start_y = self.path_cell_world(index - 1)
            end_x, end_y = self.path_cell_world(index)
            dx = end_x - start_x
            dy = end_y - start_y
            distance = math.hypot(dx, dy)
            steps = max(1, int(math.ceil(distance / self.path_interpolation_resolution)))
            yaw = math.atan2(dy, dx)

            for step in range(1, steps + 1):
                ratio = step / steps
                pose = PoseStamped()
                pose.header.frame_id = self.goal_frame_id
                pose.header.stamp = self.get_clock().now().to_msg()
                pose.pose.position.x = start_x + dx * ratio
                pose.pose.position.y = start_y + dy * ratio
                pose.pose.orientation.z = math.sin(yaw * 0.5)
                pose.pose.orientation.w = math.cos(yaw * 0.5)
                poses.append(pose)

        return poses

    def publish_grid_markers(self):
        if self.grid_msg is None:
            return

        markers = MarkerArray()
        marker = Marker()
        marker.header.frame_id = self.goal_frame_id
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'coverage_grid'
        marker.id = 0
        marker.type = Marker.LINE_LIST
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.015
        marker.color.r = 0.1
        marker.color.g = 0.4
        marker.color.b = 1.0
        marker.color.a = 0.85

        for gx in range(self.grid_width + 1):
            x0, y0 = self.grid_corner_world(gx, 0)
            x1, y1 = self.grid_corner_world(gx, self.grid_height)
            marker.points.append(self.make_point(x0, y0, 0.04))
            marker.points.append(self.make_point(x1, y1, 0.04))

        for gy in range(self.grid_height + 1):
            x0, y0 = self.grid_corner_world(0, gy)
            x1, y1 = self.grid_corner_world(self.grid_width, gy)
            marker.points.append(self.make_point(x0, y0, 0.04))
            marker.points.append(self.make_point(x1, y1, 0.04))

        markers.markers.append(marker)
        self.grid_marker_pub.publish(markers)

    def publish_priority_markers(self):
        if self.grid_msg is None or not self.coverage_path:
            return

        markers = MarkerArray()
        line = Marker()
        line.header.frame_id = self.goal_frame_id
        line.header.stamp = self.get_clock().now().to_msg()
        line.ns = 'coverage_priority'
        line.id = 0
        line.type = Marker.LINE_STRIP
        line.action = Marker.ADD
        line.pose.orientation.w = 1.0
        line.scale.x = 0.035
        line.color.r = 1.0
        line.color.g = 0.65
        line.color.b = 0.0
        line.color.a = 1.0
        safe_index = self.get_safe_zone_index()

        for index, (gx, gy) in enumerate(self.coverage_path):
            pose = self.grid_cell_to_pose(gx, gy, self.get_goal_yaw(index))
            line.points.append(self.make_point(pose.pose.position.x, pose.pose.position.y, 0.08))
            is_safe_zone = index == safe_index

            point = Marker()
            point.header.frame_id = self.goal_frame_id
            point.header.stamp = line.header.stamp
            point.ns = 'coverage_goal_points'
            point.id = 1000 + index
            point.type = Marker.SPHERE
            point.action = Marker.ADD
            point.pose = pose.pose
            point.pose.position.z = 0.08
            point.scale.x = 0.13 if is_safe_zone else 0.09
            point.scale.y = 0.13 if is_safe_zone else 0.09
            point.scale.z = 0.13 if is_safe_zone else 0.09
            point.color.r = 0.1 if is_safe_zone else 1.0
            point.color.g = 0.9 if is_safe_zone else 0.1
            point.color.b = 0.2 if is_safe_zone else 0.1
            point.color.a = 1.0
            markers.markers.append(point)

            label = Marker()
            label.header.frame_id = self.goal_frame_id
            label.header.stamp = line.header.stamp
            label.ns = 'coverage_goal_order'
            label.id = 2000 + index
            label.type = Marker.TEXT_VIEW_FACING
            label.action = Marker.ADD
            label.pose.position.x = pose.pose.position.x
            label.pose.position.y = pose.pose.position.y
            label.pose.position.z = 0.22
            label.pose.orientation.w = 1.0
            label.scale.z = 0.16
            label.color.r = 1.0
            label.color.g = 1.0
            label.color.b = 1.0
            label.color.a = 1.0
            label.text = 'SAFE %d' % (index + 1) if is_safe_zone else str(index + 1)
            markers.markers.append(label)

        markers.markers.insert(0, line)
        self.priority_marker_pub.publish(markers)

    def make_point(self, x, y, z):
        point = Point()
        point.x = x
        point.y = y
        point.z = z
        return point


def main(args=None):
    rclpy.init(args=args)
    node = CoveragePathPlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
