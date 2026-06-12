#!/usr/bin/env python3

import math
import threading
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger


class MetalGraspCoordinator(Node):
    def __init__(self):
        super().__init__('metal_grasp_coordinator')

        self.declare_parameter('metal_topic', '/metal_sensor/any_detected')
        self.declare_parameter('target_pose_topic', '/vision/target_pose')
        self.declare_parameter('motor_command_topic', '/arduino/command')
        self.declare_parameter('prepare_service_name', '/prepare_grasp_pose')
        self.declare_parameter('pick_service_name', '/pick_latest_target')
        self.declare_parameter('finish_service_name', '/finish_grasp_pose')
        self.declare_parameter('return_service_name', '/return_to_start_pose')
        self.declare_parameter('capture_service_name', '/capture_pending_target')
        self.declare_parameter('metal_hold_time_sec', 0.3)
        self.declare_parameter('pose_stable_time_sec', 0.3)
        self.declare_parameter('pose_position_tolerance_m', 0.02)
        self.declare_parameter('pose_orientation_tolerance_rad', 0.25)
        self.declare_parameter('motor_power_percent', 80)
        self.declare_parameter('motor_run_time_ms', 3000)
        self.declare_parameter('pick_cooldown_sec', 10.0)
        self.declare_parameter('recapture_target_after_return', True)
        self.declare_parameter('target_after_return_timeout_sec', 5.0)
        self.declare_parameter('target_after_return_stable_time_sec', 0.3)
        self.declare_parameter('use_pick_latest_after_return', True)

        self.metal_topic = self.get_parameter('metal_topic').value
        self.target_pose_topic = self.get_parameter('target_pose_topic').value
        self.motor_command_topic = self.get_parameter('motor_command_topic').value
        self.prepare_service_name = self.get_parameter('prepare_service_name').value
        self.pick_service_name = self.get_parameter('pick_service_name').value
        self.finish_service_name = self.get_parameter('finish_service_name').value
        self.return_service_name = self.get_parameter('return_service_name').value
        self.capture_service_name = self.get_parameter('capture_service_name').value
        self.metal_hold_time_sec = float(self.get_parameter('metal_hold_time_sec').value)
        self.pose_stable_time_sec = float(self.get_parameter('pose_stable_time_sec').value)
        self.pose_position_tolerance_m = float(self.get_parameter('pose_position_tolerance_m').value)
        self.pose_orientation_tolerance_rad = float(
            self.get_parameter('pose_orientation_tolerance_rad').value)
        self.motor_power_percent = int(self.get_parameter('motor_power_percent').value)
        self.motor_run_time_ms = int(self.get_parameter('motor_run_time_ms').value)
        self.pick_cooldown_sec = float(self.get_parameter('pick_cooldown_sec').value)
        self.recapture_target_after_return = bool(
            self.get_parameter('recapture_target_after_return').value)
        self.target_after_return_timeout_sec = float(
            self.get_parameter('target_after_return_timeout_sec').value)
        self.target_after_return_stable_time_sec = float(
            self.get_parameter('target_after_return_stable_time_sec').value)
        self.use_pick_latest_after_return = bool(
            self.get_parameter('use_pick_latest_after_return').value)

        self.any_metal = False
        self.metal_detected_time = None
        self.pose_stable_time = None
        self.last_pose_receive_time = None
        self.last_pose = None
        self.have_pose = False
        self.executing = False
        self.pose_lock = threading.Lock()
        self.last_pick_time = self.get_clock().now() - Duration(seconds=self.pick_cooldown_sec)

        self.metal_sub = self.create_subscription(
            Bool,
            self.metal_topic,
            self.on_metal_state,
            qos_profile_sensor_data,
        )
        self.pose_sub = self.create_subscription(
            PoseStamped,
            self.target_pose_topic,
            self.on_target_pose,
            qos_profile_sensor_data,
        )
        self.motor_pub = self.create_publisher(String, self.motor_command_topic, 10)

        self.prepare_client = self.create_client(Trigger, self.prepare_service_name)
        self.pick_client = self.create_client(Trigger, self.pick_service_name)
        self.finish_client = self.create_client(Trigger, self.finish_service_name)
        self.return_client = self.create_client(Trigger, self.return_service_name)
        self.capture_client = self.create_client(Trigger, self.capture_service_name)

        self.create_timer(0.1, self.control_loop)

    def on_metal_state(self, msg: Bool):
        if msg.data and not self.any_metal:
            self.metal_detected_time = self.get_clock().now()
            self.get_logger().info('Metal detection started.')

        if not msg.data and self.any_metal:
            self.get_logger().info('Metal detection cleared.')
            with self.pose_lock:
                # 금속이 끝나도 metal_detected_time은 유지하고, pose_stable_time만 초기화
                self.pose_stable_time = None

        self.any_metal = msg.data

    def on_target_pose(self, msg: PoseStamped):
        now = self.get_clock().now()
        with self.pose_lock:
            self.last_pose_receive_time = now
            if not self.have_pose:
                self.last_pose = msg
                self.pose_stable_time = now
                self.have_pose = True
                return

            if self.pose_stable_time is None:
                self.pose_stable_time = now

            if self.is_pose_changed(self.last_pose, msg):
                self.last_pose = msg
                self.pose_stable_time = now
                return

            self.last_pose = msg

    def is_pose_changed(self, a: PoseStamped, b: PoseStamped) -> bool:
        dx = a.pose.position.x - b.pose.position.x
        dy = a.pose.position.y - b.pose.position.y
        dz = a.pose.position.z - b.pose.position.z
        distance = math.sqrt(dx * dx + dy * dy + dz * dz)
        if distance > self.pose_position_tolerance_m:
            return True

        dot = (
            a.pose.orientation.x * b.pose.orientation.x +
            a.pose.orientation.y * b.pose.orientation.y +
            a.pose.orientation.z * b.pose.orientation.z +
            a.pose.orientation.w * b.pose.orientation.w)
        dot = max(-1.0, min(1.0, dot))
        angle = math.acos(min(1.0, max(-1.0, 2.0 * dot * dot - 1.0)))
        return angle > self.pose_orientation_tolerance_rad

    def control_loop(self):
        if self.executing:
            return
        if self.metal_detected_time is None:
            return
        with self.pose_lock:
            if self.last_pose is None or self.pose_stable_time is None:
                return
            pose_stable_time = self.pose_stable_time

        now = self.get_clock().now()
        elapsed_metal = (now - self.metal_detected_time).nanoseconds / 1e9
        elapsed_pose = (now - pose_stable_time).nanoseconds / 1e9
        elapsed_cooldown = (now - self.last_pick_time).nanoseconds / 1e9
        if elapsed_metal < self.metal_hold_time_sec:
            return
        if elapsed_pose < self.pose_stable_time_sec:
            return
        if elapsed_cooldown < self.pick_cooldown_sec:
            return

        self.executing = True
        self.get_logger().info('Metal detected and vision target stable. Starting integrated pick sequence.')
        threading.Thread(target=self.execute_sequence_worker, daemon=True).start()

    def execute_sequence_worker(self):
        success = self.execute_sequence()
        if success:
            self.get_logger().info('Integrated pick sequence completed successfully.')
            self.last_pick_time = self.get_clock().now()
        else:
            self.get_logger().warn('Integrated pick sequence failed.')
        self.executing = False
        self.any_metal = False
        self.metal_detected_time = None
        with self.pose_lock:
            self.pose_stable_time = None

    def execute_sequence(self) -> bool:
        if not self.call_service(self.prepare_client, 'prepare_grasp_pose'):
            self.get_logger().error('Failed to prepare grasp pose.')
            return False

        command = f'BLOW,{self.motor_power_percent},{self.motor_run_time_ms}'
        self.motor_pub.publish(String(data=command))
        self.get_logger().info(f'Sent motor command: {command}')
        # wait for motor to finish while holding current position
        time.sleep(self.motor_run_time_ms / 1000.0 + 0.2)

        # after motor finished, move arm back to start
        if not self.call_service(self.return_client, 'return_to_start_pose'):
            self.get_logger().warn('Return to start pose failed; aborting finish.')
            return False

        if not self.wait_for_fresh_target_after_return():
            return False

        if self.use_pick_latest_after_return:
            self.get_logger().info('Calling pick_latest_target after return with fresh target pose.')
            if not self.call_service(self.pick_client, 'pick_latest_target'):
                self.get_logger().error('Failed to pick latest target after return.')
                return False
            return True

        if self.recapture_target_after_return:
            if not self.call_service(self.capture_client, 'capture_pending_target'):
                self.get_logger().error('Failed to capture pending target after return.')
                return False
        else:
            self.get_logger().info('Skipping target recapture after return; using target saved during prepare.')

        # now perform finish grasp (this will open gripper then move to grasp)
        if not self.call_service(self.finish_client, 'finish_grasp_pose'):
            self.get_logger().error('Failed to finish grasp pose.')
            return False

        return True

    def wait_for_fresh_target_after_return(self) -> bool:
        start_time = self.get_clock().now()
        self.get_logger().info('Waiting for a fresh target pose after return.')

        while rclpy.ok():
            now = self.get_clock().now()
            elapsed_wait = (now - start_time).nanoseconds / 1e9
            if elapsed_wait > self.target_after_return_timeout_sec:
                self.get_logger().error(
                    f'Timed out waiting for fresh target pose after return '
                    f'({self.target_after_return_timeout_sec:.1f}s).')
                return False

            with self.pose_lock:
                pose_time = self.last_pose_receive_time
                stable_time = self.pose_stable_time

            if (
                pose_time is not None and
                stable_time is not None and
                pose_time.nanoseconds >= start_time.nanoseconds
            ):
                elapsed_stable = (now - stable_time).nanoseconds / 1e9
                if elapsed_stable >= self.target_after_return_stable_time_sec:
                    self.get_logger().info('Fresh target pose after return is stable.')
                    return True

            time.sleep(0.05)

        return False

    def call_service(self, client, label: str) -> bool:
        # allow more time for MoveIt planning/execution to become available
        if not client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error(f'Service {label} not available.')
            return False
        request = Trigger.Request()
        future = client.call_async(request)
        # Wait up to 20 seconds for the service response (poll every 0.1s)
        for i in range(200):
            if future.done():
                result = future.result()
                if result is None:
                    self.get_logger().error(f'Service {label} returned None.')
                    return False
                if not result.success:
                    self.get_logger().warn(f'Service {label} failed: {result.message}')
                    return False
                self.get_logger().info(f'Service {label} succeeded: {result.message}')
                return True
            time.sleep(0.1)
        self.get_logger().error(f'Service {label} call timeout.')
        return False


def main(args=None):
    rclpy.init(args=args)
    node = MetalGraspCoordinator()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
