#!/usr/bin/env python3
"""Initialize AMCL once before starting the autonomous mission, then verify TF."""
import json, math, os, time
from pathlib import Path
import rclpy, yaml
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rclpy.time import Time
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf2_ros import Buffer, TransformListener

root = Path(__file__).resolve().parents[1]
data = yaml.safe_load((root / "src/tb4_yolo_nav/config/goals.yaml").read_text())[
    "initial_pose"
]
rclpy.init()
node = Node(
    "lab05_initialize_pose", parameter_overrides=[Parameter("use_sim_time", value=True)]
)
pub = node.create_publisher(PoseWithCovarianceStamped, "/initialpose", 10)
buffer = Buffer()
listener = TransformListener(buffer, node)
received = []
qos = QoSProfile(
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)
node.create_subscription(
    PoseWithCovarianceStamped, "/amcl_pose", lambda m: received.append(m), qos
)
deadline = time.monotonic() + 90
sent = None
verified = None
try:
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
        if (
            sent is None
            and node.get_clock().now().nanoseconds > 0
            and pub.get_subscription_count() > 0
        ):
            msg = PoseWithCovarianceStamped()
            msg.header.frame_id = "map"
            msg.header.stamp = node.get_clock().now().to_msg()
            msg.pose.pose.position.x = float(data["x"])
            msg.pose.pose.position.y = float(data["y"])
            msg.pose.pose.orientation.z = math.sin(data["yaw"] / 2)
            msg.pose.pose.orientation.w = math.cos(data["yaw"] / 2)
            msg.pose.covariance[0] = 0.25
            msg.pose.covariance[7] = 0.25
            msg.pose.covariance[35] = 0.0685
            sent = node.get_clock().now().nanoseconds / 1e9
            pub.publish(msg)
            node.get_logger().info("Published initial pose once: " + str(data))
        if sent is None or not received:
            continue
        m = received[-1]
        stamp = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        if stamp < sent - 0.1:
            continue
        try:
            t = buffer.lookup_transform("map", "base_link", Time()).transform
        except Exception:
            continue
        q = t.rotation
        verified = {
            "passed": True,
            "initial_guess": data,
            "amcl_stamp": stamp,
            "tf_map_base": {
                "x": t.translation.x,
                "y": t.translation.y,
                "yaw": math.atan2(
                    2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)
                ),
            },
            "covariance_x": m.pose.covariance[0],
            "covariance_y": m.pose.covariance[7],
            "note": "One initialization before mission; no simulator ground truth used.",
        }
        break
    if verified is None:
        raise RuntimeError("AMCL pose/TF did not become available after initialization")
    out = root / "evidence/localization_startup.json"
    out.write_text(json.dumps(verified, indent=2) + "\n")
    print("AMCL initialized and map->base_link verified:", verified["tf_map_base"])
finally:
    node.destroy_node()
    rclpy.shutdown()
