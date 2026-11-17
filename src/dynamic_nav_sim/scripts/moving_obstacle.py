#!/usr/bin/env python3
"""Independent, position-triggered physical crossings from the scene manifest.

Ground-truth publication is optional and disabled by default.
Each obstacle follows its own clock after triggering; it never follows the robot.
"""
import json
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from ament_index_python.packages import get_package_share_directory
from gazebo_msgs.msg import ModelStates
from gazebo_msgs.srv import SetEntityState
from geometry_msgs.msg import Point32
from std_msgs.msg import String
from costmap_converter_msgs.msg import ObstacleArrayMsg, ObstacleMsg
from visualization_msgs.msg import Marker


def trajectory(elapsed, spec):
    start, speed, hold = spec['start_y'], spec['speed'], spec['hold_seconds']
    direction = -1. if start > 0. else 1.
    duration = abs(start) / speed
    if elapsed is None:
        return start, 0., 'ARMED'
    if elapsed < duration:
        return start + direction * speed * elapsed, direction * speed, 'CROSSING'
    if spec.get('motion') == 'stop_at_center':
        return 0., 0., 'STOPPED_CENTER'
    if elapsed < duration + hold:
        return 0., 0., 'BLOCKING'
    if elapsed < 2 * duration + hold:
        return direction * speed * (elapsed - duration - hold), direction * speed, 'CLEARING'
    return -start, 0., 'DONE'


class MovingObstacle(Node):
    def __init__(self):
        super().__init__('moving_obstacle')
        self.declare_parameter('publish_ground_truth', False)
        self.declare_parameter('scene', 'dynamic_crossing')
        self.declare_parameter('profile', 'responsive')
        scene = self.get_parameter('scene').value
        self.layout = json.loads((Path(get_package_share_directory('dynamic_nav_sim')) / 'config' / f'{scene}_scene.json').read_text())
        self.specs = self.layout['dynamic_obstacles']
        self.client = self.create_client(SetEntityState, '/gazebo/set_entity_state')
        self.publisher = self.create_publisher(ObstacleArrayMsg, '/obstacles', 10)
        self.marker = self.create_publisher(Marker, '/moving_obstacle_marker', 10)
        self.scenario = self.create_publisher(String, '/scenario/state', 10)
        self.create_subscription(ModelStates, '/gazebo/model_states', self.states, qos_profile_sensor_data)
        self.pending = {}
        self.trigger_times = {spec['name']: None for spec in self.specs}
        self.create_timer(.05, self.move)

    def move(self):
        if not self.client.service_is_ready():
            return
        now = self.get_clock().now().nanoseconds * 1e-9
        states = []
        for spec in self.specs:
            name = spec['name']; trigger = self.trigger_times[name]
            elapsed = None if trigger is None else max(0., now-trigger)
            y, vy, phase = trajectory(elapsed, spec)
            states.append({**spec, 'trigger_time': trigger, 'phase': phase})
            pending = self.pending.get(name)
            if pending and not pending.done():
                continue
            if pending:
                try:
                    if not pending.result().success:
                        self.get_logger().error(f'Gazebo rejected {name} state update')
                except Exception as exc:
                    self.get_logger().error(str(exc))
            request = SetEntityState.Request()
            request.state.name = name
            request.state.reference_frame = 'world'
            request.state.pose.position.x = spec['x']
            request.state.pose.position.y = y
            request.state.pose.position.z = .5
            request.state.pose.orientation.w = 1.
            request.state.twist.linear.y = vy
            self.pending[name] = self.client.call_async(request)
        phases = [item['phase'] for item in states]
        phase = 'ARMED' if all(p == 'ARMED' for p in phases) else 'STOPPED_CENTER' if all(p == 'STOPPED_CENTER' for p in phases) else 'DONE' if all(p == 'DONE' for p in phases) else next((p for p in phases if p in ('CROSSING','BLOCKING','CLEARING')), 'BETWEEN')
        # Keep legacy single-obstacle fields for existing compare_reports.py.
        first = states[0]
        self.scenario.publish(String(data=json.dumps({
            'scenario': 'multi_stage_corridor' if len(states)>1 else 'triggered_blocking_crossing',
            'phase': phase, 'scene': self.layout['name'],
            'profile': self.get_parameter('profile').value,
            'dynamic_enabled': any(spec['enabled'] for spec in self.specs),
            'sim_time': now, 'trigger_time': first['trigger_time'],
            'trigger_x': first['trigger_x'], 'radius': first['radius'],
            'start_y': first['start_y'], 'crossing_speed': first['speed'],
            'hold_seconds': first['hold_seconds'], 'obstacles': states,
            'ground_truth_to_teb': self.get_parameter('publish_ground_truth').value,
        })))

    def states(self, msg):
        now = self.get_clock().now().nanoseconds * 1e-9
        if 'demo_cart' in msg.name:
            robot = msg.pose[msg.name.index('demo_cart')]
            for spec in self.specs:
                if spec['enabled'] and self.trigger_times[spec['name']] is None and now > 0. and robot.position.x >= spec['trigger_x']:
                    self.trigger_times[spec['name']] = now
                    self.get_logger().info(f"TRIGGER {spec['name']}: robot x={robot.position.x:.3f}, crossing x={spec['x']}")
        array = ObstacleArrayMsg()
        array.header.frame_id = 'odom'
        array.header.stamp = self.get_clock().now().to_msg()
        for index, spec in enumerate(self.specs):
            if spec['name'] not in msg.name:
                continue
            idx = msg.name.index(spec['name']); pose, twist = msg.pose[idx], msg.twist[idx]
            obstacle = ObstacleMsg()
            obstacle.header = array.header
            obstacle.id = index + 1
            obstacle.radius = spec['radius']
            obstacle.orientation.w = 1.
            obstacle.polygon.points = [Point32(x=pose.position.x, y=pose.position.y, z=0.)]
            obstacle.velocities.twist = twist
            array.obstacles.append(obstacle)
            marker = Marker()
            marker.header = array.header
            marker.ns = 'physical_crossing_obstacle'
            marker.id = index + 1
            marker.type = Marker.CYLINDER
            marker.action = Marker.ADD
            marker.pose = pose
            marker.scale.x = marker.scale.y = 2*spec['radius']
            marker.scale.z = 1.
            marker.color.r, marker.color.b, marker.color.a = 1., .8 if index else 0., .8
            self.marker.publish(marker)
        if self.get_parameter('publish_ground_truth').value:
            self.publisher.publish(array)


def main():
    rclpy.init()
    node = MovingObstacle()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()

if __name__ == '__main__': main()
