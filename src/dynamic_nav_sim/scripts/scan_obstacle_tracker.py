#!/usr/bin/env python3
"""Track circular objects measured by laser; no world/model-state inputs."""
import math
from collections import deque

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener, TransformException
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Point32
from costmap_converter_msgs.msg import ObstacleArrayMsg, ObstacleMsg


def fit_circle(points):
    if len(points) < 7:
        return None
    points = np.asarray(points)
    origin = points.mean(axis=0)
    q = points - origin
    solution, _, rank, _ = np.linalg.lstsq(
        np.column_stack((2*q, np.ones(len(q)))), (q*q).sum(axis=1), rcond=None)
    if rank < 3:
        return None
    center = solution[:2]
    radius = math.sqrt(max(0., solution[2] + center @ center))
    distances = np.linalg.norm(q-center, axis=1)
    eigenvalues = np.linalg.eigvalsh(q.T @ q)
    if not .12 <= radius <= .8 or np.max(np.abs(distances-radius)) > .008:
        return None
    if eigenvalues[0] / max(eigenvalues[1], 1.e-9) < .015:
        return None  # Insufficient curvature to distinguish a wall from a circle.
    return origin + center, radius


class ScanObstacleTracker(Node):
    def __init__(self):
        super().__init__('scan_obstacle_tracker')
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, self)
        self.publisher = self.create_publisher(ObstacleArrayMsg, '/obstacles', 10)
        self.create_subscription(LaserScan, '/scan', self.enqueue_scan, qos_profile_sensor_data)
        self.tracks = {}
        self.next_id = 100
        self.pending_scans = deque(maxlen=5)
        self.last_publish_stamp = 0.
        self.create_timer(.01, self.process_pending)

    def enqueue_scan(self, scan):
        self.pending_scans.append(scan)

    def process_pending(self):
        now = self.get_clock().now().nanoseconds * 1.e-9
        # Gazebo's scan can arrive before its exact-stamp TF. Do not replace
        # that transform with the latest pose or discard every such scan.
        while self.pending_scans:
            scan = self.pending_scans[0]
            stamp = scan.header.stamp.sec + scan.header.stamp.nanosec * 1.e-9
            if now-stamp > .25:
                self.pending_scans.popleft()
            elif self.scan(scan):
                self.pending_scans.popleft()
            else:
                break
        # TEB retains the last custom obstacle array until another arrives.
        # Expire it even when no scan can be transformed. Live costmap obstacle
        # collision checks remain active independently of these predictions.
        if now-self.last_publish_stamp > .6:
            output = ObstacleArrayMsg()
            output.header.frame_id = 'odom'
            output.header.stamp = self.get_clock().now().to_msg()
            self.publisher.publish(output)
            self.tracks.clear()
            self.last_publish_stamp = now

    def scan(self, scan):
        try:
            transform = self.tf.lookup_transform('odom', scan.header.frame_id,
                                                  Time.from_msg(scan.header.stamp))
        except TransformException:
            return False
        t = scan.header.stamp.sec + scan.header.stamp.nanosec * 1.e-9
        tr = transform.transform.translation
        q = transform.transform.rotation
        yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
        c, s = math.cos(yaw), math.sin(yaw)
        rotation = np.array([[c,-s],[s,c]])
        groups, group = [], []
        for i, distance in enumerate(scan.ranges):
            if not math.isfinite(distance) or not scan.range_min <= distance <= 5.:
                if group: groups.append(group)
                group = []
                continue
            angle = scan.angle_min + i*scan.angle_increment
            point = np.array([distance*math.cos(angle),distance*math.sin(angle)])
            if group and np.linalg.norm(point-group[-1]) > .12 + .015*distance:
                groups.append(group)
                group = []
            group.append(point)
        if group: groups.append(group)
        detections = []
        for group in groups:
            fit = fit_circle(group)
            if fit is not None:
                center, radius = fit
                detections.append((rotation @ center + [tr.x,tr.y],radius))
        available = set(self.tracks)
        for center, radius in detections:
            candidates = [(np.linalg.norm(center-(self.tracks[k]['position'] +
                          self.tracks[k]['velocity']*(t-self.tracks[k]['time']))),k)
                          for k in available if t-self.tracks[k]['time'] < .7]
            distance, key = min(candidates,default=(float('inf'),None))
            if distance > .35:
                key = self.next_id
                self.next_id += 1
                self.tracks[key] = {'history':deque(maxlen=4),'velocity':np.zeros(2)}
            else:
                available.remove(key)
            track = self.tracks[key]
            track['history'].append((t,center))
            track.update(position=center,radius=radius,time=t)
            history = track['history']
            if len(history) >= 3 and t-history[0][0] >= .18:
                times = np.array([item[0]-t for item in history])
                points = np.array([item[1] for item in history])
                velocity = np.linalg.lstsq(np.column_stack((times,np.ones(len(times)))),points,rcond=None)[0][0]
                speed = np.linalg.norm(velocity)
                track['velocity'] = velocity if .08 < speed < 1.5 else np.zeros(2)
        output = ObstacleArrayMsg()
        output.header = scan.header
        output.header.frame_id = 'odom'
        for key,track in list(self.tracks.items()):
            age = t-track['time']
            if age > .6:
                del self.tracks[key]
                continue
            if len(track['history']) < 3:
                continue
            center = track['position'] + track['velocity']*age
            obstacle = ObstacleMsg()
            obstacle.id = key
            obstacle.polygon.points = [Point32(x=float(center[0]),y=float(center[1]))]
            # Small measurement + latency allowance. Future motion belongs in
            # the time-dependent velocity model, not extra stationary circles.
            obstacle.radius = float(track['radius'] + .06 +
                                    min(.10, .10*np.linalg.norm(track['velocity']) + .2*age))
            obstacle.orientation.w = 1.
            obstacle.velocities.twist.linear.x = float(track['velocity'][0])
            obstacle.velocities.twist.linear.y = float(track['velocity'][1])
            output.obstacles.append(obstacle)
        self.publisher.publish(output)
        self.last_publish_stamp = t
        return True


def main():
    rclpy.init()
    node = ScanObstacleTracker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()


if __name__ == '__main__':
    main()
