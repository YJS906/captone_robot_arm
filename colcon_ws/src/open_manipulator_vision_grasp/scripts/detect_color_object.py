#!/usr/bin/env python3

import math
from collections import deque
from typing import Optional

import cv2
from cv_bridge import CvBridge
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from geometry_msgs.msg import PointStamped, PoseStamped
import tf2_geometry_msgs  # noqa: F401
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker


class ColorObjectDetector(Node):
    def __init__(self):
        super().__init__('detect_color_object')
        self.bridge = CvBridge()
        self.depth_image: Optional[np.ndarray] = None
        self.camera_info: Optional[CameraInfo] = None

        self.declare_parameter('color_topic', '/camera/color/image_raw')
        self.declare_parameter('depth_topic', '/camera/depth/image_raw')
        self.declare_parameter('camera_info_topic', '/camera/depth/camera_info')
        self.declare_parameter('output_frame', 'link1')
        self.declare_parameter('target_pose_topic', '/vision/target_pose')
        self.declare_parameter('marker_topic', '/vision/target_marker')
        self.declare_parameter('debug_image_topic', '/vision/debug_image')
        self.declare_parameter('min_depth_m', 0.12)
        self.declare_parameter('max_depth_m', 0.60)
        self.declare_parameter('min_area_px', 500.0)
        self.declare_parameter('sample_radius_px', 5)
        self.declare_parameter('hsv_lower', [0, 80, 60])
        self.declare_parameter('hsv_upper', [12, 255, 255])
        self.declare_parameter('target_orientation_xyzw', [0.0, 0.0, 0.0, 1.0])
        # Shape filter parameters (for rectangular / cuboid detection)
        self.declare_parameter('shape_filter_enabled', True)
        self.declare_parameter('min_aspect_ratio', 0.7)
        self.declare_parameter('max_aspect_ratio', 1.5)
        self.declare_parameter('min_extent', 0.6)
        self.declare_parameter('max_extent', 1.0)
        self.declare_parameter('min_solidity', 0.85)
        self.declare_parameter('max_solidity', 1.0)
        self.declare_parameter('min_corner_count', 4)
        self.declare_parameter('max_corner_count', 6)
        self.declare_parameter('pose_filter_enabled', True)
        self.declare_parameter('pose_filter_window_size', 7)
        self.declare_parameter('pose_filter_min_samples', 5)
        self.declare_parameter('pose_filter_max_spread_m', 0.015)

        self.output_frame = self.get_parameter('output_frame').value
        self.pose_filter_enabled = bool(self.get_parameter('pose_filter_enabled').value)
        self.pose_filter_window_size = max(1, int(self.get_parameter('pose_filter_window_size').value))
        self.pose_filter_min_samples = max(1, int(self.get_parameter('pose_filter_min_samples').value))
        self.pose_filter_min_samples = min(self.pose_filter_min_samples, self.pose_filter_window_size)
        self.pose_filter_max_spread_m = float(self.get_parameter('pose_filter_max_spread_m').value)
        self.pose_buffer = deque(maxlen=self.pose_filter_window_size)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.pose_pub = self.create_publisher(
            PoseStamped,
            self.get_parameter('target_pose_topic').value,
            10,
        )
        self.marker_pub = self.create_publisher(
            Marker,
            self.get_parameter('marker_topic').value,
            10,
        )
        self.debug_pub = self.create_publisher(
            Image,
            self.get_parameter('debug_image_topic').value,
            10,
        )

        self.create_subscription(
            CameraInfo,
            self.get_parameter('camera_info_topic').value,
            self._on_camera_info,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Image,
            self.get_parameter('depth_topic').value,
            self._on_depth,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Image,
            self.get_parameter('color_topic').value,
            self._on_color,
            qos_profile_sensor_data,
        )

    def _on_camera_info(self, msg: CameraInfo):
        self.camera_info = msg

    def _on_depth(self, msg: Image):
        self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

    def _on_color(self, msg: Image):
        color = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        if self.depth_image is None or self.camera_info is None:
            self._publish_debug(color, status='waiting for depth/camera_info')
            return

        hsv = cv2.cvtColor(color, cv2.COLOR_BGR2HSV)
        lower = np.array(self.get_parameter('hsv_lower').value, dtype=np.uint8)
        upper = np.array(self.get_parameter('hsv_upper').value, dtype=np.uint8)
        if int(lower[0]) <= int(upper[0]):
            mask = cv2.inRange(hsv, lower, upper)
        else:
            lower_high = lower.copy()
            upper_high = upper.copy()
            lower_low = lower.copy()
            upper_low = upper.copy()
            upper_high[0] = 179
            lower_low[0] = 0
            mask = cv2.bitwise_or(
                cv2.inRange(hsv, lower_high, upper_high),
                cv2.inRange(hsv, lower_low, upper_low))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((7, 7), np.uint8))

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            self._reset_pose_filter()
            self._publish_debug(color, status='no red contour')
            return

        contour = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(contour)
        if area < float(self.get_parameter('min_area_px').value):
            self._reset_pose_filter()
            self._publish_debug(color, contour, status=f'area too small: {area:.0f}')
            return

        # Optional shape filtering: reject non-rectangular contours
        if self.get_parameter('shape_filter_enabled').value:
            metrics = self._contour_metrics(contour)
            if metrics is None:
                self._reset_pose_filter()
                self._publish_debug(color, contour, status='shape metrics failed')
                return
            # Check area again (metric-derived) and shape constraints
            min_area = float(self.get_parameter('min_area_px').value)
            if metrics['area'] < min_area:
                self._reset_pose_filter()
                self._publish_debug(color, contour, status=f"area too small: {metrics['area']:.0f}")
                return
            aspect = metrics['aspect_ratio']
            extent = metrics['extent']
            solidity = metrics['solidity']
            corners = metrics['corners']
            if not (float(self.get_parameter('min_aspect_ratio').value) <= aspect <= float(self.get_parameter('max_aspect_ratio').value)):
                self._reset_pose_filter()
                self._publish_debug(color, contour, status=f'aspect rejected: {aspect:.2f}')
                return
            if not (float(self.get_parameter('min_extent').value) <= extent <= float(self.get_parameter('max_extent').value)):
                self._reset_pose_filter()
                self._publish_debug(color, contour, status=f'extent rejected: {extent:.2f}')
                return
            if not (float(self.get_parameter('min_solidity').value) <= solidity <= float(self.get_parameter('max_solidity').value)):
                self._reset_pose_filter()
                self._publish_debug(color, contour, status=f'solidity rejected: {solidity:.2f}')
                return
            if not (int(self.get_parameter('min_corner_count').value) <= corners <= int(self.get_parameter('max_corner_count').value)):
                self._reset_pose_filter()
                self._publish_debug(color, contour, status=f'corners rejected: {corners}')
                return

        moments = cv2.moments(contour)
        if abs(moments['m00']) < 1e-6:
            return
        u = int(moments['m10'] / moments['m00'])
        v = int(moments['m01'] / moments['m00'])

        depth_m = self._sample_depth_m(u, v)
        if depth_m is None:
            self._reset_pose_filter()
            self._publish_debug(color, contour, u, v, None, status='no valid depth')
            return

        point = self._project_pixel_to_point(u, v, depth_m, self.camera_info)
        if point is None:
            self._reset_pose_filter()
            return

        pose = PoseStamped()
        pose.header = point.header
        pose.pose.position.x = point.point.x
        pose.pose.position.y = point.point.y
        pose.pose.position.z = point.point.z
        q = self.get_parameter('target_orientation_xyzw').value
        pose.pose.orientation.x = float(q[0])
        pose.pose.orientation.y = float(q[1])
        pose.pose.orientation.z = float(q[2])
        pose.pose.orientation.w = float(q[3])

        if self.output_frame:
            try:
                point.header.stamp.sec = 0
                point.header.stamp.nanosec = 0
                point = self.tf_buffer.transform(point, self.output_frame, timeout=rclpy.duration.Duration(seconds=0.05))
                pose.header = point.header
                pose.pose.position.x = point.point.x
                pose.pose.position.y = point.point.y
                pose.pose.position.z = point.point.z
            except TransformException as exc:
                self._reset_pose_filter()
                self.get_logger().warn(f'TF transform failed: {exc}', throttle_duration_sec=2.0)
                return

        filtered_pose, spread_m = self._filter_pose(pose)
        if filtered_pose is None:
            self._publish_debug(
                color,
                contour,
                u,
                v,
                depth_m,
                status=f'stabilizing pose {len(self.pose_buffer)}/{self.pose_filter_min_samples}')
            return

        self.pose_pub.publish(filtered_pose)
        self._publish_marker(filtered_pose)
        self._publish_debug(color, contour, u, v, depth_m, status=f'target stable {spread_m * 1000.0:.0f}mm')

    def _reset_pose_filter(self):
        if self.pose_buffer:
            self.pose_buffer.clear()

    def _filter_pose(self, pose: PoseStamped):
        if not self.pose_filter_enabled:
            return pose, 0.0

        self.pose_buffer.append((
            pose.pose.position.x,
            pose.pose.position.y,
            pose.pose.position.z,
        ))

        if len(self.pose_buffer) < self.pose_filter_min_samples:
            return None, math.inf

        samples = np.array(self.pose_buffer, dtype=np.float64)
        median = np.median(samples, axis=0)
        distances = np.linalg.norm(samples - median, axis=1)
        spread_m = float(np.max(distances))
        if spread_m > self.pose_filter_max_spread_m:
            return None, spread_m

        filtered_pose = PoseStamped()
        filtered_pose.header = pose.header
        filtered_pose.pose.position.x = float(median[0])
        filtered_pose.pose.position.y = float(median[1])
        filtered_pose.pose.position.z = float(median[2])
        filtered_pose.pose.orientation = pose.pose.orientation
        return filtered_pose, spread_m

    def _sample_depth_m(self, u: int, v: int) -> Optional[float]:
        depth = self.depth_image
        if depth is None:
            return None
        radius = int(self.get_parameter('sample_radius_px').value)
        y0 = max(0, v - radius)
        y1 = min(depth.shape[0], v + radius + 1)
        x0 = max(0, u - radius)
        x1 = min(depth.shape[1], u + radius + 1)
        sample = depth[y0:y1, x0:x1].astype(np.float32)
        if depth.dtype == np.uint16:
            sample *= 0.001
        valid = sample[np.isfinite(sample)]
        min_depth = float(self.get_parameter('min_depth_m').value)
        max_depth = float(self.get_parameter('max_depth_m').value)
        valid = valid[(valid >= min_depth) & (valid <= max_depth)]
        if valid.size == 0:
            return None
        return float(np.median(valid))

    def _project_pixel_to_point(self, u: int, v: int, z: float, info: CameraInfo) -> Optional[PointStamped]:
        fx = info.k[0]
        fy = info.k[4]
        cx = info.k[2]
        cy = info.k[5]
        if fx == 0.0 or fy == 0.0 or not math.isfinite(z):
            return None
        point = PointStamped()
        point.header = info.header
        point.point.x = (float(u) - cx) * z / fx
        point.point.y = (float(v) - cy) * z / fy
        point.point.z = z
        return point

    def _contour_metrics(self, contour):
        area = float(cv2.contourArea(contour))
        x, y, w, h = cv2.boundingRect(contour)
        if w <= 0 or h <= 0:
            return None
        hull = cv2.convexHull(contour)
        hull_area = float(cv2.contourArea(hull))
        if hull_area <= 0.0:
            return None
        aspect_ratio = float(h) / float(w)
        extent = area / float(w * h) if (w * h) > 0 else 0.0
        solidity = area / hull_area
        approx = cv2.approxPolyDP(contour, 0.02 * cv2.arcLength(contour, True), True)
        corners = int(len(approx))
        return {
            'area': area,
            'aspect_ratio': aspect_ratio,
            'extent': extent,
            'solidity': solidity,
            'corners': corners,
        }

    def _publish_marker(self, pose: PoseStamped):
        marker = Marker()
        marker.header = pose.header
        marker.ns = 'vision_target'
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose = pose.pose
        marker.scale.x = 0.03
        marker.scale.y = 0.03
        marker.scale.z = 0.03
        marker.color.r = 0.1
        marker.color.g = 0.9
        marker.color.b = 0.2
        marker.color.a = 0.9
        self.marker_pub.publish(marker)

    def _publish_debug(self, image, contour=None, u=None, v=None, depth_m=None, status=''):
        debug = image.copy()
        if contour is not None:
            cv2.drawContours(debug, [contour], -1, (0, 255, 0), 2)
        if u is not None and v is not None:
            cv2.circle(debug, (u, v), 5, (255, 0, 0), -1)
            label = 'no depth' if depth_m is None else f'{depth_m:.3f} m'
            cv2.putText(debug, label, (u + 8, v - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        if status:
            cv2.putText(
                debug,
                status,
                (12, 24),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (255, 255, 255),
                2)
        self.debug_pub.publish(self.bridge.cv2_to_imgmsg(debug, encoding='bgr8'))


def main():
    rclpy.init()
    node = ColorObjectDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
