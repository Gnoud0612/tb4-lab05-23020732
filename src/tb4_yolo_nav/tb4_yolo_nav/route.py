"""Validate the explicit operator-selected map goals, without planning or motion."""

import math
from pathlib import Path
import yaml


def load_route(path):
    data = yaml.safe_load(Path(path).read_text())
    if not isinstance(data, dict) or data.get("frame_id") != "map":
        raise ValueError("Route frame_id must be map")
    goals = data.get("goals", [])
    if not isinstance(goals, list) or len(goals) < 3:
        raise ValueError("At least three map goals are required")
    names = set()
    positions = []
    for pose in [data.get("initial_pose", {}), *goals]:
        if not isinstance(pose, dict):
            raise ValueError("Each pose must be a mapping")
        for key in ["x", "y", "yaw"]:
            value = pose.get(key)
            if (
                isinstance(value, bool)
                or not isinstance(value, (int, float))
                or not math.isfinite(value)
            ):
                raise ValueError(f"Pose {key} must be a finite number")
    for goal in goals:
        name = goal.get("name")
        if not isinstance(name, str) or not name or name in names:
            raise ValueError("Goal names must be nonempty and unique")
        if any(math.hypot(goal["x"] - x, goal["y"] - y) < 1.0 for x, y in positions):
            raise ValueError("Goals must be at least 1 m apart")
        names.add(name)
        positions.append((goal["x"], goal["y"]))
    for key, default in [
        ("dwell_wall_seconds", 10.0),
        ("goal_timeout_wall_seconds", 900.0),
        ("startup_timeout_wall_seconds", 180.0),
    ]:
        value = data.get(key, default)
        if (
            isinstance(value, bool)
            or not isinstance(value, (int, float))
            or not math.isfinite(value)
            or value < 0
            or (key != "dwell_wall_seconds" and value == 0)
        ):
            raise ValueError(f"Invalid {key}")
        data[key] = float(value)
    return data
