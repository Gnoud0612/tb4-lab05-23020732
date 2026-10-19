"""Passively record Nav2 action statuses and AMCL poses without commanding motion."""

import argparse
from datetime import datetime, timezone
import json
import math
import os
from pathlib import Path
import tempfile
import time

from action_msgs.msg import GoalStatus, GoalStatusArray
from geometry_msgs.msg import PoseWithCovarianceStamped
from irobot_create_msgs.msg import HazardDetection, HazardDetectionVector
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data


STATUS_NAMES = {
    GoalStatus.STATUS_UNKNOWN: 'UNKNOWN', GoalStatus.STATUS_ACCEPTED: 'ACCEPTED',
    GoalStatus.STATUS_EXECUTING: 'EXECUTING', GoalStatus.STATUS_CANCELING: 'CANCELING',
    GoalStatus.STATUS_SUCCEEDED: 'SUCCEEDED', GoalStatus.STATUS_CANCELED: 'CANCELED',
    GoalStatus.STATUS_ABORTED: 'ABORTED',
}
ACTIVE = {GoalStatus.STATUS_ACCEPTED, GoalStatus.STATUS_EXECUTING, GoalStatus.STATUS_CANCELING}
TERMINAL = {GoalStatus.STATUS_SUCCEEDED, GoalStatus.STATUS_CANCELED, GoalStatus.STATUS_ABORTED}


def stamp_seconds(stamp):
    return stamp.sec + stamp.nanosec / 1e9


class GoalMonitor(Node):
    def __init__(self, options):
        super().__init__('tb4_lab_goal_monitor', parameter_overrides=[
            Parameter('use_sim_time', value=True)])
        self.options = options
        self.output = Path(options.output).expanduser().resolve()
        self.started_wall = time.monotonic()
        self.started_utc = datetime.now(timezone.utc).isoformat()
        self.goals = {}
        self.active_ids = set()
        self.latest_pose = None
        self.cmd_publishers = []
        self.publisher_history = []
        self.initial_status_received = False
        self.next_target = 0
        self.hazard_message_count = 0
        self.bump_keys = {}
        status_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                                durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(GoalStatusArray, '/navigate_to_pose/_action/status',
                                 self.on_status, status_qos)
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.on_pose, QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(HazardDetectionVector, '/hazard_detection',
                                 self.on_hazards, qos_profile_sensor_data)

    def simulation_time(self):
        return self.get_clock().now().nanoseconds / 1e9

    def on_pose(self, message):
        pose = message.pose.pose
        q = pose.orientation
        values = [pose.position.x, pose.position.y, q.x, q.y, q.z, q.w]
        if not all(math.isfinite(value) for value in values):
            return
        self.latest_pose = {
            'frame_id': message.header.frame_id,
            'stamp_seconds': stamp_seconds(message.header.stamp),
            'x': pose.position.x, 'y': pose.position.y,
            'yaw_radians': math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                                      1.0 - 2.0 * (q.y * q.y + q.z * q.z)),
            'covariance_x': (message.pose.covariance[0]
                             if math.isfinite(message.pose.covariance[0]) else None),
            'covariance_y': (message.pose.covariance[7]
                             if math.isfinite(message.pose.covariance[7]) else None),
            'covariance_yaw': (message.pose.covariance[35]
                               if math.isfinite(message.pose.covariance[35]) else None),
        }

    def poll_publishers(self):
        publishers = sorted([
            {'node_name': endpoint.node_name, 'node_namespace': endpoint.node_namespace,
             'type': endpoint.topic_type}
            for endpoint in self.get_publishers_info_by_topic('/cmd_vel')
        ], key=lambda item: (item['node_namespace'], item['node_name'], item['type']))
        if publishers != self.cmd_publishers or not self.publisher_history:
            self.publisher_history.append({
                'elapsed_wall_seconds': time.monotonic() - self.started_wall,
                'simulation_time_seconds': self.simulation_time(), 'publishers': publishers})
            self.cmd_publishers = publishers
        teleop = [item for item in publishers if 'teleop' in item['node_name'].lower()]
        for goal_id in self.active_ids:
            goal = self.goals[goal_id]
            for publisher in publishers:
                if publisher not in goal['cmd_vel_publishers_observed_while_active']:
                    goal['cmd_vel_publishers_observed_while_active'].append(publisher)
            if teleop:
                goal['teleop_publisher_present_while_active'] = True

    def on_hazards(self, message):
        self.hazard_message_count += 1
        elapsed = time.monotonic() - self.started_wall
        first_bump_seen = False
        for goal_id in self.active_ids:
            goal = self.goals[goal_id]
            goal['hazard_messages_received_while_active'] += 1
            for detection in message.detections:
                if detection.type != HazardDetection.BUMP:
                    continue
                key = (detection.header.stamp.sec, detection.header.stamp.nanosec,
                       detection.header.frame_id)
                if key in self.bump_keys[goal_id]:
                    continue
                self.bump_keys[goal_id].add(key)
                first_bump_seen = first_bump_seen or not goal['bump_detected']
                goal['bump_detected'] = True
                goal['unique_bump_detection_message_count'] += 1
                goal['unique_bump_detection_messages'].append({
                    'type': int(HazardDetection.BUMP),
                    'frame_id': detection.header.frame_id,
                    'stamp_seconds': stamp_seconds(detection.header.stamp),
                    'vector_stamp_seconds': stamp_seconds(message.header.stamp),
                    'received_elapsed_wall_seconds': elapsed,
                    'received_simulation_time_seconds': self.simulation_time(),
                })
        if first_bump_seen:
            self.save()

    def on_status(self, message):
        # Attribute the graph snapshot to goals active before this transition too.
        self.poll_publishers()
        active_ids = set()
        terminal_changed = False
        for status in message.status_list:
            goal_id = bytes(status.goal_info.goal_id.uuid).hex()
            now_wall = time.monotonic()
            now_ros = self.simulation_time()
            goal = self.goals.get(goal_id)
            if goal is None:
                historical = not self.initial_status_received and status.status in TERMINAL
                goal = {
                    'goal_id': goal_id,
                    'goal_stamp_seconds': stamp_seconds(status.goal_info.stamp),
                    'first_seen_elapsed_wall_seconds': now_wall - self.started_wall,
                    'first_seen_simulation_time_seconds': now_ros,
                    'initial_terminal_history': historical,
                    'observed_active_status': False,
                    'status_history': [],
                    'terminal_status': None,
                    'final_amcl_pose': None,
                    'teleop_publisher_present_while_active': False,
                    'cmd_vel_publishers_observed_while_active': [],
                    'hazard_messages_received_while_active': 0,
                    'bump_detected': False,
                    'unique_bump_detection_message_count': 0,
                    'unique_bump_detection_messages': [],
                }
                if not historical and self.next_target < len(self.options.target):
                    x, y = self.options.target[self.next_target]
                    goal['operator_declared_target'] = {
                        'frame_id': 'map', 'x': x, 'y': y,
                        'source': 'CLI metadata; not read from the action request'}
                    self.next_target += 1
                self.goals[goal_id] = goal
                self.bump_keys[goal_id] = set()
            name = STATUS_NAMES.get(status.status, f'UNRECOGNIZED_{status.status}')
            if not goal['status_history'] or goal['status_history'][-1]['status'] != name:
                goal['status_history'].append({
                    'status': name, 'elapsed_wall_seconds': now_wall - self.started_wall,
                    'simulation_time_seconds': now_ros})
            if status.status in ACTIVE:
                active_ids.add(goal_id)
                goal['observed_active_status'] = True
            if status.status in TERMINAL and goal['terminal_status'] is None:
                goal['terminal_status'] = name
                goal['completed_elapsed_wall_seconds'] = now_wall - self.started_wall
                goal['observed_duration_wall_seconds'] = (
                    goal['completed_elapsed_wall_seconds'] - goal['first_seen_elapsed_wall_seconds'])
                start_stamp = goal['goal_stamp_seconds']
                goal['duration_simulation_seconds'] = (
                    now_ros - start_stamp if start_stamp > 0 and now_ros >= start_stamp else None)
                if not goal['initial_terminal_history'] and self.latest_pose:
                    goal['final_amcl_pose'] = dict(self.latest_pose)
                    goal['final_amcl_pose']['age_at_terminal_seconds'] = (
                        now_ros - self.latest_pose['stamp_seconds'])
                terminal_changed = True
                kind = 'HISTORICAL' if goal['initial_terminal_history'] else 'OBSERVED'
                print(f'{kind} {goal_id[:12]}: {name}', flush=True)
        self.active_ids = active_ids
        self.initial_status_received = True
        self.poll_publishers()
        if terminal_changed:
            self.save()

    def report(self):
        goals = list(self.goals.values())
        successes = [goal for goal in goals if goal['terminal_status'] == 'SUCCEEDED'
                     and not goal['initial_terminal_history']]
        # Greedy separated positions are evidence of distinct destinations, not coverage.
        distinct = []
        for goal in successes:
            pose = goal['final_amcl_pose']
            if not pose or pose['frame_id'] != 'map':
                continue
            if all(math.hypot(pose['x'] - other['x'], pose['y'] - other['y'])
                   >= self.options.minimum_separation for other in distinct):
                distinct.append({'x': pose['x'], 'y': pose['y'], 'goal_id': goal['goal_id']})
        status_position_met = (len(successes) >= self.options.required_goals
                               and len(distinct) >= self.options.required_goals)
        teleop_overlap = any(goal['teleop_publisher_present_while_active'] for goal in goals)
        bump_detected = any(goal['bump_detected'] for goal in goals)
        hazard_observed = bool(successes) and all(
            goal['hazard_messages_received_while_active'] > 0 for goal in successes)
        criteria_met = (status_position_met and not teleop_overlap and not bump_detected
                        and hazard_observed)
        return {
            'schema_version': 1, 'label': self.options.label,
            'started_at_utc': self.started_utc,
            'updated_at_utc': datetime.now(timezone.utc).isoformat(),
            'duration_wall_seconds': time.monotonic() - self.started_wall,
            'summary': {
                'successful_goal_count': len(successes),
                'distinct_final_positions': distinct,
                'minimum_separation_metres': self.options.minimum_separation,
                'required_goals': self.options.required_goals,
                'status_and_position_criteria_met': status_position_met,
                'monitor_criteria_met': criteria_met,
                'teleop_publisher_overlap_detected': teleop_overlap,
                'bump_detected': bump_detected,
                'unique_bump_detection_message_count': sum(
                    len(goal['unique_bump_detection_messages']) for goal in goals),
                'hazard_message_count_total': self.hazard_message_count,
                'hazard_stream_observed_for_each_successful_goal': hazard_observed,
                'collision_free_navigation': 'BUMP reports sampled; visual review still required',
            },
            'goals': goals,
            'cmd_vel_publisher_history': self.publisher_history,
            'limitations': [
                'Action status reports the server state; it does not reveal the target pose or full result.',
                'Final pose is the most recent AMCL sample at the terminal status, with its age recorded.',
                'A teleop publisher may exist without sending commands; absence is limited to discovery samples.',
                'Collision avoidance and RViz Goal use must be demonstrated in the video.',
                'No BUMP reports is only a sampled hazard signal, not a guarantee that no contact occurred.',
                'BUMP messages are deduplicated by sensor frame and detection timestamp; counts are not collision episodes.',
                'Terminal statuses retained before monitoring began are excluded from the success count.',
            ],
        }

    def save(self):
        self.output.parent.mkdir(parents=True, exist_ok=True)
        with tempfile.NamedTemporaryFile(mode='w', encoding='utf-8',
                                         dir=self.output.parent, delete=False) as stream:
            temporary = stream.name
            json.dump(self.report(), stream, indent=2, allow_nan=False)
            stream.write('\n')
        os.replace(temporary, self.output)


def main(args=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', default='evidence/goals.json', help='JSON evidence file')
    parser.add_argument('--label', default='RViz Nav2 goals', help='Human-readable recording label')
    parser.add_argument('--duration', type=float, default=0.0,
                        help='Wall seconds to observe; 0 runs until Ctrl+C')
    parser.add_argument('--required-goals', type=int, default=3)
    parser.add_argument('--minimum-separation', type=float, default=0.5,
                        help='Minimum separation in metres for distinct final map positions')
    parser.add_argument('--target', action='append', nargs=2, type=float, default=[], metavar=('X', 'Y'),
                        help='Optional manually declared map target; repeat in expected goal order')
    options, ros_args = parser.parse_known_args(args)
    if options.duration < 0 or not math.isfinite(options.duration):
        parser.error('--duration must be finite and nonnegative')
    if options.required_goals < 1:
        parser.error('--required-goals must be positive')
    if options.minimum_separation <= 0 or not math.isfinite(options.minimum_separation):
        parser.error('--minimum-separation must be finite and positive')
    if any(not math.isfinite(value) for target in options.target for value in target):
        parser.error('--target values must be finite')
    rclpy.init(args=ros_args)
    node = GoalMonitor(options)
    last_graph = last_save = 0.0
    print(f'Passive monitor started; evidence: {node.output}', flush=True)
    try:
        node.poll_publishers()
        node.save()
        while rclpy.ok():
            now = time.monotonic()
            if options.duration and now - node.started_wall >= options.duration:
                break
            rclpy.spin_once(node, timeout_sec=0.1)
            if now - last_graph >= 0.5:
                node.poll_publishers()
                last_graph = now
            if now - last_save >= 5.0:
                node.save()
                last_save = now
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.save()
        summary = node.report()['summary']
        print(f"Saved {summary['successful_goal_count']} successful goals, "
              f"{len(summary['distinct_final_positions'])} distinct final positions to {node.output}")
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    raise SystemExit(0 if summary['monitor_criteria_met'] else 1)


if __name__ == '__main__':
    main()
