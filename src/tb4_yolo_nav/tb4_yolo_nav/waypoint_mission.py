"""Send three or more sequential NavigateToPose actions. Never publish cmd_vel.

No ground truth subscription, teleoperation, or pose reset after navigation starts.
Failures cancel the current task and stop the mission; no failed goal is skipped.
"""

import json, math, os, signal, time
from datetime import datetime, timezone
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.action import ActionClient
from rclpy.clock import Clock, ClockType
from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    DurabilityPolicy,
    qos_profile_sensor_data,
)
from rclpy.signals import SignalHandlerOptions
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus
from irobot_create_msgs.msg import HazardDetectionVector, HazardDetection
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray
from .route import load_route


def now_utc():
    return datetime.now(timezone.utc).isoformat()


class WaypointMission(Node):
    def __init__(self):
        super().__init__(
            "waypoint_mission",
            parameter_overrides=[Parameter("use_sim_time", value=True)],
        )
        base = Path(os.environ.get("TB4_WORKSPACE", "."))
        default = base / "src/tb4_yolo_nav/config/goals.yaml"
        self.declare_parameter("route_file", str(default))
        self.declare_parameter("output", str(base / "evidence/mission.json"))
        self.declare_parameter("initialize_pose", False)
        self.route = load_route(self.get_parameter("route_file").value)
        self.output = Path(self.get_parameter("output").value)
        self.started = time.monotonic()
        self.state = "WAIT_SERVERS"
        self.done = False
        self.success = False
        self.index = 0
        self.latest_pose = None
        self.pose_sent = None
        self.goal_handle = None
        self.active_record = None
        self.goal_sent_wall = None
        self.dwell_until = 0
        self.last_feedback_log = 0
        self.last_save = 0
        self.last_marker = 0
        self.cancel_future = None
        self.log = {
            "started_at_utc": now_utc(),
            "route": self.route,
            "goals": [],
            "events": [],
            "passed": False,
            "source": "Real NavigateToPose action client",
            "policy": "No cmd_vel publisher, no manual goals, no pose resets during mission",
        }
        self.client = ActionClient(self, NavigateToPose, "/navigate_to_pose")
        latched = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, "/initialpose", 10
        )
        self.status_pub = self.create_publisher(String, "/mission/status", latched)
        self.marker_pub = self.create_publisher(MarkerArray, "/mission/goals", latched)
        self.create_subscription(
            PoseWithCovarianceStamped, "/amcl_pose", self.on_pose, latched
        )
        self.create_subscription(
            HazardDetectionVector,
            "/hazard_detection",
            self.on_hazard,
            qos_profile_sensor_data,
        )
        self.lifecycle = {
            name: self.create_client(GetState, f"/{name}/get_state")
            for name in [
                "map_server",
                "amcl",
                "planner_server",
                "controller_server",
                "bt_navigator",
            ]
        }
        self.lifecycle_pending = {}
        self.lifecycle_active = set()
        self.timer = self.create_timer(
            0.2, self.tick, clock=Clock(clock_type=ClockType.STEADY_TIME)
        )
        self.event("mission_started", goals=len(self.route["goals"]))

    def event(self, kind, **kwargs):
        entry = {
            "event": kind,
            "utc": now_utc(),
            "elapsed_wall_seconds": time.monotonic() - self.started,
            "simulation_time_seconds": self.get_clock().now().nanoseconds / 1e9,
            **kwargs,
        }
        self.log["events"].append(entry)
        self.get_logger().info(json.dumps(entry, ensure_ascii=False))
        self.publish_status()
        self.save()

    def on_pose(self, m):
        q = m.pose.pose.orientation
        p = m.pose.pose.position
        values = [p.x, p.y, q.x, q.y, q.z, q.w, *m.pose.covariance]
        if not all(math.isfinite(v) for v in values) or m.header.frame_id != "map":
            return
        self.latest_pose = {
            "x": p.x,
            "y": p.y,
            "yaw": math.atan2(
                2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)
            ),
            "stamp": m.header.stamp.sec + m.header.stamp.nanosec * 1e-9,
            "covariance_x": m.pose.covariance[0],
            "covariance_y": m.pose.covariance[7],
            "covariance_yaw": m.pose.covariance[35],
        }

    def on_hazard(self, m):
        if self.state != "EXECUTING" or self.active_record is None:
            return
        self.active_record["hazard_messages"] += 1
        if any(h.type == HazardDetection.BUMP for h in m.detections):
            self.active_record["bump_detected"] = True
            self.fail("BUMP reported while navigating")

    def make_pose(self, data):
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = float(data["x"])
        pose.pose.position.y = float(data["y"])
        pose.pose.orientation.z = math.sin(data["yaw"] / 2)
        pose.pose.orientation.w = math.cos(data["yaw"] / 2)
        return pose

    def poll_lifecycle(self, required=None):
        for name, client in self.lifecycle.items():
            if name in self.lifecycle_active:
                continue
            future = self.lifecycle_pending.get(name)
            if future is not None:
                if not future.done():
                    continue
                try:
                    response = future.result()
                    if response.current_state.id == 3:
                        self.lifecycle_active.add(name)
                except Exception as e:
                    self.get_logger().warning(f"{name}: {e}")
                self.lifecycle_pending.pop(name, None)
            elif client.service_is_ready():
                self.lifecycle_pending[name] = client.call_async(GetState.Request())
        return set(required or self.lifecycle).issubset(self.lifecycle_active)

    def tick(self):
        if self.done:
            return
        wall = time.monotonic()
        elapsed = wall - self.started
        if wall - self.last_marker > 1:
            self.publish_markers()
            self.last_marker = wall
        if wall - self.last_save > 2:
            self.save()
            self.last_save = wall
        if (
            self.state in ["WAIT_SERVERS", "WAIT_LOCALIZATION"]
            and elapsed > self.route["startup_timeout_wall_seconds"]
        ):
            return self.fail("Startup/localization timeout")
        if self.state == "WAIT_SERVERS":
            if (
                not self.poll_lifecycle(["map_server", "amcl"])
                or self.get_clock().now().nanoseconds == 0
            ):
                return
            self.pose_sent = wall
            self.pose_sent_ros = self.get_clock().now().nanoseconds / 1e9
            if self.get_parameter("initialize_pose").value:
                msg = PoseWithCovarianceStamped()
                pose = self.make_pose(self.route["initial_pose"])
                msg.header = pose.header
                msg.pose.pose = pose.pose
                msg.pose.covariance[0] = 0.25
                msg.pose.covariance[7] = 0.25
                msg.pose.covariance[35] = 0.0685
                self.pose_pub.publish(msg)
                self.event("initial_pose_sent", pose=self.route["initial_pose"])
            self.state = "WAIT_LOCALIZATION"
        elif self.state == "WAIT_LOCALIZATION":
            pose = self.latest_pose
            if pose is None or wall - self.pose_sent < 5:
                return
            if (
                self.get_parameter("initialize_pose").value
                and pose["stamp"] < self.pose_sent_ros - 0.1
            ):
                return
            if not self.poll_lifecycle() or not self.client.server_is_ready():
                return
            if (
                pose["covariance_x"] > 0.8
                or pose["covariance_y"] > 0.8
                or pose["covariance_yaw"] > 0.5
            ):
                return
            self.event("localization_ready", amcl=pose)
            self.send_next()
        elif self.state == "SENDING" and wall - self.goal_sent_wall > 30:
            self.fail("Action acceptance timeout")
        elif self.state == "EXECUTING":
            if wall - self.goal_sent_wall > self.route["goal_timeout_wall_seconds"]:
                self.fail("Goal wall-time timeout")
            else:
                teleop = [
                    x.node_name
                    for x in self.get_publishers_info_by_topic("/cmd_vel")
                    if "teleop" in x.node_name.lower()
                ]
                if teleop:
                    self.fail(
                        "Teleoperation publisher detected while executing: "
                        + str(teleop)
                    )
        elif self.state == "DWELL" and wall >= self.dwell_until:
            self.send_next()
        elif self.state == "CANCELING":
            if self.cancel_future is not None and self.cancel_future.done():
                self.event("cancel_response_received")
            if wall - self.cancel_started > 8:
                self.finish(False)

    def send_next(self):
        if self.index >= len(self.route["goals"]):
            return self.finish(True)
        data = self.route["goals"][self.index]
        request = NavigateToPose.Goal()
        request.pose = self.make_pose(data)
        record = {
            "index": self.index + 1,
            "target": data,
            "sent_at_utc": now_utc(),
            "sent_elapsed_wall_seconds": time.monotonic() - self.started,
            "terminal_status": None,
            "hazard_messages": 0,
            "bump_detected": False,
        }
        self.log["goals"].append(record)
        self.active_record = record
        self.state = "SENDING"
        self.goal_sent_wall = time.monotonic()
        self.event("goal_sent", index=self.index + 1, target=data)
        future = self.client.send_goal_async(
            request, feedback_callback=self.on_feedback
        )
        future.add_done_callback(self.on_accepted)

    def on_accepted(self, future):
        try:
            self.goal_handle = future.result()
        except Exception as e:
            return self.fail("Sending goal failed: " + str(e))
        if not self.goal_handle.accepted:
            return self.fail("Nav2 rejected the goal")
        self.active_record["goal_id"] = bytes(self.goal_handle.goal_id.uuid).hex()
        if self.state == "CANCELING":
            self.cancel_future = self.goal_handle.cancel_goal_async()
            return
        self.state = "EXECUTING"
        self.event(
            "goal_accepted", index=self.index + 1, goal_id=self.active_record["goal_id"]
        )
        self.goal_handle.get_result_async().add_done_callback(self.on_result)

    def on_feedback(self, msg):
        feedback = msg.feedback
        if time.monotonic() - self.last_feedback_log < 3:
            return
        self.last_feedback_log = time.monotonic()
        self.active_record["last_feedback"] = {
            "distance_remaining": float(feedback.distance_remaining),
            "number_of_recoveries": int(feedback.number_of_recoveries),
        }
        self.publish_status()
        self.get_logger().info(
            f'Goal {self.index+1}/{len(self.route["goals"])} distance={feedback.distance_remaining:.2f} m recoveries={feedback.number_of_recoveries}'
        )

    def on_result(self, future):
        try:
            result = future.result()
        except Exception as e:
            return self.fail("Result failed: " + str(e))
        status = {
            GoalStatus.STATUS_SUCCEEDED: "SUCCEEDED",
            GoalStatus.STATUS_ABORTED: "ABORTED",
            GoalStatus.STATUS_CANCELED: "CANCELED",
        }.get(result.status, str(result.status))
        rec = self.active_record
        rec["terminal_status"] = status
        rec["completed_at_utc"] = now_utc()
        rec["observed_duration_wall_seconds"] = time.monotonic() - self.goal_sent_wall
        rec["final_amcl_pose"] = self.latest_pose
        self.event("goal_result", index=self.index + 1, status=status)
        if self.state == "CANCELING":
            return self.finish(False)
        if result.status != GoalStatus.STATUS_SUCCEEDED:
            return self.fail("Navigation did not succeed: " + status)
        self.index += 1
        self.goal_handle = None
        self.state = "DWELL"
        self.dwell_until = time.monotonic() + self.route["dwell_wall_seconds"]
        self.event(
            "goal_dwell", completed=self.index, seconds=self.route["dwell_wall_seconds"]
        )

    def fail(self, reason):
        if self.done or self.state == "CANCELING":
            return
        self.log["failure_reason"] = reason
        self.event("mission_failed", reason=reason)
        if self.goal_handle is not None and self.state == "EXECUTING":
            self.state = "CANCELING"
            self.cancel_started = time.monotonic()
            self.cancel_future = self.goal_handle.cancel_goal_async()
        elif self.state == "SENDING":
            self.state = "CANCELING"
            self.cancel_started = time.monotonic()
        else:
            self.finish(False)

    def finish(self, passed):
        self.success = bool(passed)
        self.done = True
        self.state = "SUCCEEDED" if passed else "FAILED"
        self.log["passed"] = self.success
        self.log["finished_at_utc"] = now_utc()
        self.publish_markers()
        self.event("mission_finished", passed=self.success)

    def publish_status(self):
        msg = String()
        msg.data = json.dumps(
            {
                "state": self.state,
                "completed": self.index,
                "required": len(self.route["goals"]),
                "current_goal": self.index + 1,
                "target": (
                    self.route["goals"][self.index]
                    if self.index < len(self.route["goals"])
                    else None
                ),
                "feedback": (
                    self.active_record.get("last_feedback")
                    if self.active_record
                    else None
                ),
                "amcl": self.latest_pose,
            },
            ensure_ascii=False,
        )
        self.status_pub.publish(msg)

    def publish_markers(self):
        arr = MarkerArray()
        for i, g in enumerate(self.route["goals"]):
            for text in [False, True]:
                m = Marker()
                m.header.frame_id = "map"
                m.header.stamp = self.get_clock().now().to_msg()
                m.ns = "mission_goal_labels" if text else "mission_goals"
                m.id = i
                m.type = Marker.TEXT_VIEW_FACING if text else Marker.ARROW
                m.action = Marker.ADD
                m.pose = self.make_pose(g).pose
                m.pose.position.z = 0.6 if text else 0.1
                if text:
                    m.text = f'G{i+1} ({g["x"]:.1f}, {g["y"]:.1f})'
                    m.scale.z = 0.35
                else:
                    m.scale.x = 0.65
                    m.scale.y = 0.12
                    m.scale.z = 0.12
                m.color.a = 1.0
                m.color.g = 1.0 if i < self.index else 0.4
                m.color.r = 0.1 if i < self.index else 1.0
                m.color.b = 0.1
                arr.markers.append(m)
        self.marker_pub.publish(arr)

    def save(self):
        self.output.parent.mkdir(parents=True, exist_ok=True)
        self.log["updated_at_utc"] = now_utc()
        self.log["state"] = self.state
        temp = self.output.with_suffix(".json.tmp")
        temp.write_text(json.dumps(self.log, indent=2, ensure_ascii=False) + "\n")
        temp.replace(self.output)


def main(args=None):
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = WaypointMission()

    def stop(*_):
        node.fail("Operator interruption")

    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
    except Exception as e:
        node.fail(str(e))
        raise
    finally:
        node.save()
        passed = node.success
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    raise SystemExit(0 if passed else 1)


if __name__ == "__main__":
    main()
