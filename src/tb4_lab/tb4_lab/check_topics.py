"""Read-only checks of the sensor, command and TF interfaces of the simulator."""

import argparse
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformException, TransformListener


def stamp_seconds(stamp):
    return stamp.sec + stamp.nanosec / 1e9


class Samples:
    def __init__(self):
        self.count = 0
        self.valid = 0
        self.stamps = []
        self.arrivals = []
        self.frames = set()

    def add(self, stamp=None, frame=None, valid=True):
        self.count += 1
        self.valid += int(valid)
        self.arrivals.append(time.monotonic())
        if stamp is not None:
            self.stamps.append(stamp)
        if frame:
            self.frames.add(frame)

    def report(self):
        wall_span = self.arrivals[-1] - self.arrivals[0] if self.count > 1 else 0.0
        ros_span = self.stamps[-1] - self.stamps[0] if len(self.stamps) > 1 else 0.0
        return {
            'message_count': self.count,
            'valid_message_count': self.valid,
            'wall_rate_hz': (self.count - 1) / wall_span if wall_span > 0 else None,
            'simulation_rate_hz': (len(self.stamps) - 1) / ros_span if ros_span > 0 else None,
            'first_stamp_seconds': self.stamps[0] if self.stamps else None,
            'last_stamp_seconds': self.stamps[-1] if self.stamps else None,
            'advancing_stamps': ros_span > 0,
            'frames': sorted(self.frames),
        }


class TopicChecks(Node):
    def __init__(self, mapping):
        super().__init__('tb4_lab_check_topics', parameter_overrides=[
            Parameter('use_sim_time', value=True)])
        self.mapping = mapping
        self.samples = {name: Samples() for name in ['/scan', '/odom', '/cmd_vel', '/clock']}
        self.child_frames = set()
        self.finite_scan_ranges = 0
        self.latest_odom = None
        self.map_info = None
        self.create_subscription(LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)
        self.create_subscription(Odometry, '/odom', self.on_odom, qos_profile_sensor_data)
        self.create_subscription(Twist, '/cmd_vel', self.on_cmd_vel, 10)
        self.create_subscription(Clock, '/clock', self.on_clock, qos_profile_sensor_data)
        if mapping:
            self.samples['/map'] = Samples()
            map_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                 durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self.create_subscription(OccupancyGrid, '/map', self.on_map, map_qos)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def on_scan(self, message):
        stamp = stamp_seconds(message.header.stamp)
        finite = sum(math.isfinite(value) and message.range_min <= value <= message.range_max
                     for value in message.ranges)
        valid = bool(message.header.frame_id) and stamp > 0 and finite > 0
        self.finite_scan_ranges += finite
        self.samples['/scan'].add(stamp, message.header.frame_id, valid)

    def on_odom(self, message):
        stamp = stamp_seconds(message.header.stamp)
        pose = message.pose.pose
        twist = message.twist.twist
        values = [pose.position.x, pose.position.y, pose.position.z,
                  pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w,
                  twist.linear.x, twist.linear.y, twist.linear.z,
                  twist.angular.x, twist.angular.y, twist.angular.z]
        quaternion_norm = sum(value * value for value in values[3:7])
        valid = (stamp > 0 and bool(message.header.frame_id) and bool(message.child_frame_id)
                 and all(math.isfinite(value) for value in values)
                 and abs(quaternion_norm - 1.0) < 0.1)
        self.samples['/odom'].add(stamp, message.header.frame_id, valid)
        self.child_frames.add(message.child_frame_id)
        self.latest_odom = {
            name: value if math.isfinite(value) else None
            for name, value in [('x', pose.position.x), ('y', pose.position.y),
                                ('linear_x', twist.linear.x), ('angular_z', twist.angular.z)]}

    def on_cmd_vel(self, message):
        values = [message.linear.x, message.linear.y, message.linear.z,
                  message.angular.x, message.angular.y, message.angular.z]
        self.samples['/cmd_vel'].add(valid=all(math.isfinite(value) for value in values))

    def on_clock(self, message):
        stamp = stamp_seconds(message.clock)
        self.samples['/clock'].add(stamp, valid=stamp > 0)

    def on_map(self, message):
        expected_size = message.info.width * message.info.height
        known = sum(value >= 0 for value in message.data)
        valid = (expected_size > 0 and len(message.data) == expected_size and known > 0
                 and message.info.resolution > 0 and bool(message.header.frame_id))
        self.samples['/map'].add(stamp_seconds(message.header.stamp), message.header.frame_id, valid)
        self.map_info = {'width': message.info.width, 'height': message.info.height,
                         'resolution': (message.info.resolution
                                        if math.isfinite(message.info.resolution) else None),
                         'known_cells': known,
                         'occupied_cells': sum(value >= 65 for value in message.data)}

    def endpoints(self, topic):
        def describe(endpoint):
            return {'node_name': endpoint.node_name,
                    'node_namespace': endpoint.node_namespace, 'type': endpoint.topic_type}
        return {
            'publishers': [describe(item) for item in self.get_publishers_info_by_topic(topic)],
            'subscriptions': [describe(item) for item in self.get_subscriptions_info_by_topic(topic)],
        }

    def transform(self, frame):
        try:
            tf = self.tf_buffer.lookup_transform(frame, 'base_link', Time())
            value = tf.transform
            values = [value.translation.x, value.translation.y, value.translation.z,
                      value.rotation.x, value.rotation.y, value.rotation.z, value.rotation.w]
            if not all(math.isfinite(item) for item in values):
                return {'available': False, 'error': 'Transform contains a nonfinite value'}
            return {'available': True, 'stamp_seconds': stamp_seconds(tf.header.stamp),
                    'translation': {'x': value.translation.x, 'y': value.translation.y,
                                    'z': value.translation.z},
                    'rotation': {'x': value.rotation.x, 'y': value.rotation.y,
                                 'z': value.rotation.z, 'w': value.rotation.w}}
        except TransformException as error:
            return {'available': False, 'error': str(error)}

    def report(self, duration):
        topics = {name: sample.report() for name, sample in self.samples.items()}
        for name, topic in topics.items():
            topic.update(self.endpoints(name))
        topics['/scan']['finite_in_range_measurements'] = self.finite_scan_ranges
        topics['/odom']['child_frames'] = sorted(self.child_frames)
        topics['/odom']['last_pose_and_velocity'] = self.latest_odom
        if self.mapping:
            topics['/map']['map_info'] = self.map_info
        transforms = {f'{frame}->base_link': self.transform(frame) for frame in ['odom', 'map']}
        failures = []
        for name in ['/scan', '/odom', '/clock']:
            if topics[name]['valid_message_count'] < 2:
                failures.append(f'{name}: need at least two valid messages')
            if not topics[name]['advancing_stamps']:
                failures.append(f'{name}: timestamps are not advancing')
        command_subscribers = [item for item in topics['/cmd_vel']['subscriptions']
                               if item['node_name'] != self.get_name()]
        if not any(item['type'] == 'geometry_msgs/msg/Twist' for item in command_subscribers):
            failures.append('/cmd_vel: no robot subscriber of geometry_msgs/msg/Twist discovered')
        if not transforms['odom->base_link']['available']:
            failures.append('TF odom->base_link is unavailable')
        if self.mapping:
            if topics['/map']['valid_message_count'] < 1:
                failures.append('/map: no valid occupancy grid received')
            if not transforms['map->base_link']['available']:
                failures.append('TF map->base_link is unavailable')
        return {
            'schema_version': 1,
            'recorded_at_utc': datetime.now(timezone.utc).isoformat(),
            'duration_wall_seconds': duration, 'mapping_required': self.mapping,
            'passed': not failures, 'failures': failures, 'topics': topics, 'tf': transforms,
            'limitations': [
                'A cmd_vel subscriber proves the command interface exists; test motion separately.',
                'Topic and TF checks do not prove mapping coverage or collision-free navigation.',
            ],
        }


def main(args=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--duration', type=float, default=10.0, help='Observation time in wall seconds')
    parser.add_argument('--mapping', action='store_true', help='Also require a map and map->base_link TF')
    parser.add_argument('--output', default='evidence/topics.json', help='JSON evidence file')
    options, ros_args = parser.parse_known_args(args)
    if options.duration <= 0 or not math.isfinite(options.duration):
        parser.error('--duration must be finite and positive')
    rclpy.init(args=ros_args)
    node = TopicChecks(options.mapping)
    started = time.monotonic()
    try:
        while rclpy.ok() and time.monotonic() - started < options.duration:
            rclpy.spin_once(node, timeout_sec=0.1)
        report = node.report(time.monotonic() - started)
        path = Path(options.output).expanduser().resolve()
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(json.dumps(report, indent=2, allow_nan=False) + '\n', encoding='utf-8')
        print(f"{'PASS' if report['passed'] else 'FAIL'}: {path}")
        for failure in report['failures']:
            print(f'  - {failure}')
        return_code = 0 if report['passed'] else 1
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    raise SystemExit(return_code)


if __name__ == '__main__':
    main()
