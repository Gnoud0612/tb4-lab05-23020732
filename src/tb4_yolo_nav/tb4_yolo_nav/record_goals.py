"""Record RViz Publish Point positions with an operator-selected yaw."""

import math
from pathlib import Path
import rclpy, yaml
from rclpy.node import Node
from geometry_msgs.msg import PointStamped


class Recorder(Node):
    def __init__(self):
        super().__init__("goal_recorder")
        self.declare_parameter("yaw", 0.0)
        self.declare_parameter("output", "recorded_goals.yaml")
        self.goals = []
        self.create_subscription(PointStamped, "/clicked_point", self.point, 10)
        self.get_logger().info(
            "Use Publish Point in RViz. yaw parameter is radians; append to output YAML."
        )

    def point(self, msg):
        if msg.header.frame_id != "map":
            return self.get_logger().error("Set RViz Fixed Frame to map")
        yaw = float(self.get_parameter("yaw").value)
        if not all(math.isfinite(v) for v in [msg.point.x, msg.point.y, yaw]):
            return
        self.goals.append(
            {
                "name": f"Goal_{len(self.goals)+1}",
                "x": msg.point.x,
                "y": msg.point.y,
                "yaw": yaw,
            }
        )
        out = Path(self.get_parameter("output").value)
        out.parent.mkdir(parents=True, exist_ok=True)
        out.write_text(
            yaml.safe_dump(
                {
                    "frame_id": "map",
                    "initial_pose": {"x": -0.35, "y": 0.0, "yaw": math.pi},
                    "goals": self.goals,
                },
                sort_keys=False,
            )
        )
        self.get_logger().info(str(self.goals[-1]))


def main(args=None):
    rclpy.init(args=args)
    node = Recorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
