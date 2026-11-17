#!/usr/bin/env python3
"""Remove returns inside the generic demo chassis, without clearing occluded directions."""
import json
import math
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from ament_index_python.packages import get_package_share_directory


class SelfScanFilter(Node):
    def __init__(self):
        super().__init__('filter_self_scan')
        config = Path(get_package_share_directory('dynamic_nav_sim')) / 'config'
        model = json.loads((config / 'robot_geometry.json').read_text())
        sensor = json.loads((config / 'generic_sensors.json').read_text())
        self.lx, self.ly, _ = sensor['lidar']['optical_xyz_in_sim_base_m']
        footprint = model['footprint_with_2cm_margin']
        # Remove the planning margin: only the physical vehicle envelope is masked.
        self.xmin = min(p[0] for p in footprint) + .02
        self.xmax = max(p[0] for p in footprint) - .02
        self.ymin = min(p[1] for p in footprint) + .02
        self.ymax = max(p[1] for p in footprint) - .02
        self.publisher = self.create_publisher(LaserScan, '/scan', qos_profile_sensor_data)
        self.create_subscription(LaserScan, '/scan_raw', self.scan, qos_profile_sensor_data)

    def scan(self, msg):
        values = list(msg.ranges)
        for i, distance in enumerate(values):
            if not math.isfinite(distance):
                continue
            angle = msg.angle_min + i * msg.angle_increment
            x = self.lx + distance * math.cos(angle)
            y = self.ly + distance * math.sin(angle)
            if self.xmin <= x <= self.xmax and self.ymin <= y <= self.ymax:
                values[i] = float('nan')
        msg.ranges = values
        self.publisher.publish(msg)


def main():
    rclpy.init()
    node = SelfScanFilter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
