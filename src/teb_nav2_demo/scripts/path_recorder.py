#!/usr/bin/env python3

import json
import math
import os
from typing import Dict, List, Optional, Sequence, Tuple

import rclpy
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node


Point2D = Tuple[float, float]


def pose_to_xy(pose) -> Point2D:
    return (float(pose.position.x), float(pose.position.y))


def stamp_to_float(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) / 1e9


def distance(a: Point2D, b: Point2D) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])


def distance_to_segment(point: Point2D, start: Point2D, end: Point2D) -> float:
    dx = end[0] - start[0]
    dy = end[1] - start[1]
    seg_len_sq = dx * dx + dy * dy
    if seg_len_sq <= 1e-12:
        return distance(point, start)
    ratio = ((point[0] - start[0]) * dx + (point[1] - start[1]) * dy) / seg_len_sq
    ratio = max(0.0, min(1.0, ratio))
    projection = (start[0] + ratio * dx, start[1] + ratio * dy)
    return distance(point, projection)


def distance_to_polyline(point: Point2D, polyline: Sequence[Point2D]) -> Optional[float]:
    if not polyline:
        return None
    if len(polyline) == 1:
        return distance(point, polyline[0])
    best = None
    for start, end in zip(polyline[:-1], polyline[1:]):
        current = distance_to_segment(point, start, end)
        if best is None or current < best:
            best = current
    return best


def yaw_to_quaternion(yaw: float) -> Tuple[float, float, float, float]:
    half = yaw * 0.5
    return (0.0, 0.0, math.sin(half), math.cos(half))


def quaternion_multiply(
    left: Tuple[float, float, float, float],
    right: Tuple[float, float, float, float],
) -> Tuple[float, float, float, float]:
    lx, ly, lz, lw = left
    rx, ry, rz, rw = right
    return (
        lw * rx + lx * rw + ly * rz - lz * ry,
        lw * ry - lx * rz + ly * rw + lz * rx,
        lw * rz + lx * ry - ly * rx + lz * rw,
        lw * rw - lx * rx - ly * ry - lz * rz,
    )


class PathRecorder(Node):
    def __init__(self) -> None:
        super().__init__('path_recorder')

        self.declare_parameter('output_dir', '')
        self.declare_parameter('actual_path_topic', '/teb_debug/actual_path')
        self.declare_parameter('plan_topic', '/plan')
        self.declare_parameter('local_plan_topic', '/local_plan')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('cmd_vel_nav_topic', '/cmd_vel_nav')
        self.declare_parameter('min_record_distance', 0.02)
        self.declare_parameter('map_offset_x', 0.0)
        self.declare_parameter('map_offset_y', 0.0)
        self.declare_parameter('map_offset_yaw', 0.0)

        self.output_dir = self.get_parameter('output_dir').get_parameter_value().string_value
        self.actual_path_topic = self.get_parameter('actual_path_topic').get_parameter_value().string_value
        self.plan_topic = self.get_parameter('plan_topic').get_parameter_value().string_value
        self.local_plan_topic = self.get_parameter('local_plan_topic').get_parameter_value().string_value
        self.odom_topic = self.get_parameter('odom_topic').get_parameter_value().string_value
        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').get_parameter_value().string_value
        self.cmd_vel_nav_topic = self.get_parameter('cmd_vel_nav_topic').get_parameter_value().string_value
        self.min_record_distance = (
            self.get_parameter('min_record_distance').get_parameter_value().double_value
        )
        self.map_offset_x = self.get_parameter('map_offset_x').get_parameter_value().double_value
        self.map_offset_y = self.get_parameter('map_offset_y').get_parameter_value().double_value
        self.map_offset_yaw = self.get_parameter('map_offset_yaw').get_parameter_value().double_value

        if not self.output_dir:
            raise RuntimeError('parameter "output_dir" must not be empty')
        os.makedirs(self.output_dir, exist_ok=True)

        self.actual_path_msg = Path()
        self.actual_path_points: List[Dict[str, float]] = []
        self.actual_path_pub = self.create_publisher(Path, self.actual_path_topic, 10)
        self.plan_snapshots: List[Dict[str, object]] = []
        self.local_plan_snapshots: List[Dict[str, object]] = []
        self.deviation_samples: List[Dict[str, float]] = []
        self.cmd_vel_samples: List[Dict[str, float]] = []
        self.cmd_vel_nav_samples: List[Dict[str, float]] = []
        self.latest_plan_points: List[Point2D] = []
        self.latest_plan_frame = 'map'
        self.latest_local_plan_points: List[Point2D] = []
        self.plan_update_count = 0
        self.local_plan_update_count = 0

        self.create_subscription(Path, self.plan_topic, self.plan_callback, 10)
        self.create_subscription(Path, self.local_plan_topic, self.local_plan_callback, 10)
        self.create_subscription(Odometry, self.odom_topic, self.odom_callback, 50)
        self.create_subscription(Twist, self.cmd_vel_topic, self.cmd_vel_callback, 50)
        self.create_subscription(Twist, self.cmd_vel_nav_topic, self.cmd_vel_nav_callback, 50)

        self.get_logger().info(f'writing path logs to {self.output_dir}')

    def point_to_map(self, point: Point2D, frame_id: str) -> Point2D:
        if frame_id == 'odom':
            cos_yaw = math.cos(self.map_offset_yaw)
            sin_yaw = math.sin(self.map_offset_yaw)
            return (
                self.map_offset_x + cos_yaw * point[0] - sin_yaw * point[1],
                self.map_offset_y + sin_yaw * point[0] + cos_yaw * point[1],
            )
        return point

    def path_msg_to_snapshot(self, msg: Path, path_id: int) -> Dict[str, object]:
        poses = []
        for pose_stamped in msg.poses:
            point = self.point_to_map(pose_to_xy(pose_stamped.pose), msg.header.frame_id)
            poses.append(
                {
                    'x': point[0],
                    'y': point[1],
                    'z': float(pose_stamped.pose.position.z),
                }
            )
        return {
            'id': path_id,
            'received_time': self.get_clock().now().nanoseconds / 1e9,
            'header_stamp': stamp_to_float(msg.header.stamp),
            'frame_id': 'map',
            'poses': poses,
        }

    def plan_callback(self, msg: Path) -> None:
        self.plan_update_count += 1
        self.latest_plan_points = [self.point_to_map(pose_to_xy(p.pose), msg.header.frame_id) for p in msg.poses]
        self.latest_plan_frame = 'map'
        self.plan_snapshots.append(self.path_msg_to_snapshot(msg, self.plan_update_count))

    def local_plan_callback(self, msg: Path) -> None:
        self.local_plan_update_count += 1
        self.latest_local_plan_points = [
            self.point_to_map(pose_to_xy(p.pose), msg.header.frame_id) for p in msg.poses
        ]
        self.local_plan_snapshots.append(self.path_msg_to_snapshot(msg, self.local_plan_update_count))

    def odom_callback(self, msg: Odometry) -> None:
        point = self.point_to_map(pose_to_xy(msg.pose.pose), msg.header.frame_id)
        if self.actual_path_points:
            previous = self.actual_path_points[-1]
            if distance(point, (previous['x'], previous['y'])) < self.min_record_distance:
                return

        timestamp = stamp_to_float(msg.header.stamp)
        record = {
            'time': timestamp,
            'x': point[0],
            'y': point[1],
            'z': float(msg.pose.pose.position.z),
        }
        self.actual_path_points.append(record)

        pose_stamped = PoseStamped()
        pose_stamped.header.stamp = msg.header.stamp
        pose_stamped.header.frame_id = 'map'
        pose_stamped.pose.position.x = point[0]
        pose_stamped.pose.position.y = point[1]
        pose_stamped.pose.position.z = float(msg.pose.pose.position.z)
        odom_orientation = msg.pose.pose.orientation
        offset_orientation = yaw_to_quaternion(self.map_offset_yaw)
        composed = quaternion_multiply(
            offset_orientation,
            (
                odom_orientation.x,
                odom_orientation.y,
                odom_orientation.z,
                odom_orientation.w,
            ),
        )
        pose_stamped.pose.orientation.x = composed[0]
        pose_stamped.pose.orientation.y = composed[1]
        pose_stamped.pose.orientation.z = composed[2]
        pose_stamped.pose.orientation.w = composed[3]

        self.actual_path_msg.header.frame_id = 'map'
        self.actual_path_msg.header.stamp = msg.header.stamp
        self.actual_path_msg.poses.append(pose_stamped)
        self.actual_path_pub.publish(self.actual_path_msg)

        if self.latest_plan_points:
            plan_deviation = distance_to_polyline(point, self.latest_plan_points)
            local_plan_deviation = distance_to_polyline(point, self.latest_local_plan_points)
            self.deviation_samples.append(
                {
                    'time': timestamp,
                    'x': point[0],
                    'y': point[1],
                    'distance_to_plan': -1.0 if plan_deviation is None else plan_deviation,
                    'distance_to_local_plan': -1.0
                    if local_plan_deviation is None
                    else local_plan_deviation,
                    'plan_update_id': float(self.plan_update_count),
                    'local_plan_update_id': float(self.local_plan_update_count),
                }
            )

    def twist_to_row(self, msg: Twist) -> Dict[str, float]:
        return {
            'time': self.get_clock().now().nanoseconds / 1e9,
            'linear_x': float(msg.linear.x),
            'linear_y': float(msg.linear.y),
            'linear_z': float(msg.linear.z),
            'angular_x': float(msg.angular.x),
            'angular_y': float(msg.angular.y),
            'angular_z': float(msg.angular.z),
        }

    def cmd_vel_callback(self, msg: Twist) -> None:
        self.cmd_vel_samples.append(self.twist_to_row(msg))

    def cmd_vel_nav_callback(self, msg: Twist) -> None:
        self.cmd_vel_nav_samples.append(self.twist_to_row(msg))

    def write_json(self, filename: str, data: object) -> None:
        path = os.path.join(self.output_dir, filename)
        with open(path, 'w', encoding='utf-8') as file:
            json.dump(data, file, ensure_ascii=True, indent=2)

    def write_csv(self, filename: str, header: Sequence[str], rows: Sequence[Sequence[object]]) -> None:
        path = os.path.join(self.output_dir, filename)
        with open(path, 'w', encoding='utf-8') as file:
            file.write(','.join(header) + '\n')
            for row in rows:
                file.write(','.join(str(item) for item in row) + '\n')

    def build_metrics(self) -> Dict[str, object]:
        valid_plan_distances = [
            sample['distance_to_plan']
            for sample in self.deviation_samples
            if sample['distance_to_plan'] >= 0.0
        ]
        valid_local_plan_distances = [
            sample['distance_to_local_plan']
            for sample in self.deviation_samples
            if sample['distance_to_local_plan'] >= 0.0
        ]

        def summarize(values: Sequence[float]) -> Dict[str, Optional[float]]:
            if not values:
                return {'count': 0, 'mean': None, 'max': None, 'rmse': None}
            mean = sum(values) / len(values)
            rmse = math.sqrt(sum(value * value for value in values) / len(values))
            return {
                'count': len(values),
                'mean': mean,
                'max': max(values),
                'rmse': rmse,
            }

        return {
            'plan_updates': self.plan_update_count,
            'local_plan_updates': self.local_plan_update_count,
            'actual_path_points': len(self.actual_path_points),
            'global_plan_deviation': summarize(valid_plan_distances),
            'local_plan_deviation': summarize(valid_local_plan_distances),
        }

    def flush(self) -> None:
        actual_rows = [
            (point['time'], point['x'], point['y'], point['z']) for point in self.actual_path_points
        ]
        self.write_csv('actual_path.csv', ('time', 'x', 'y', 'z'), actual_rows)

        deviation_rows = [
            (
                sample['time'],
                sample['x'],
                sample['y'],
                sample['distance_to_plan'],
                sample['distance_to_local_plan'],
                int(sample['plan_update_id']),
                int(sample['local_plan_update_id']),
            )
            for sample in self.deviation_samples
        ]
        self.write_csv(
            'path_deviation.csv',
            (
                'time',
                'x',
                'y',
                'distance_to_plan',
                'distance_to_local_plan',
                'plan_update_id',
                'local_plan_update_id',
            ),
            deviation_rows,
        )

        cmd_vel_rows = [
            (
                sample['time'],
                sample['linear_x'],
                sample['linear_y'],
                sample['linear_z'],
                sample['angular_x'],
                sample['angular_y'],
                sample['angular_z'],
            )
            for sample in self.cmd_vel_samples
        ]
        self.write_csv(
            'cmd_vel.csv',
            (
                'time',
                'linear_x',
                'linear_y',
                'linear_z',
                'angular_x',
                'angular_y',
                'angular_z',
            ),
            cmd_vel_rows,
        )

        cmd_vel_nav_rows = [
            (
                sample['time'],
                sample['linear_x'],
                sample['linear_y'],
                sample['linear_z'],
                sample['angular_x'],
                sample['angular_y'],
                sample['angular_z'],
            )
            for sample in self.cmd_vel_nav_samples
        ]
        self.write_csv(
            'cmd_vel_nav.csv',
            (
                'time',
                'linear_x',
                'linear_y',
                'linear_z',
                'angular_x',
                'angular_y',
                'angular_z',
            ),
            cmd_vel_nav_rows,
        )

        latest_plan = self.plan_snapshots[-1] if self.plan_snapshots else None
        latest_local_plan = self.local_plan_snapshots[-1] if self.local_plan_snapshots else None
        self.write_json('plan_history.json', self.plan_snapshots)
        self.write_json('local_plan_history.json', self.local_plan_snapshots)
        self.write_json('planned_path_latest.json', latest_plan)
        self.write_json('local_path_latest.json', latest_local_plan)
        self.write_json('path_tracking_metrics.json', self.build_metrics())


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PathRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.flush()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
