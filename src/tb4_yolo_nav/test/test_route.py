"""Reject unsafe/ambiguous route input before any robot action is sent."""

import copy
from pathlib import Path
import pytest
import yaml
from tb4_yolo_nav.route import load_route


def valid():
    return {
        "frame_id": "map",
        "initial_pose": {"x": 0.0, "y": 0.0, "yaw": 0.0},
        "goals": [
            {"name": str(i), "x": float(i * 3), "y": 0.0, "yaw": 0.0} for i in range(3)
        ],
    }


def write(tmp_path, data):
    p = tmp_path / "route.yaml"
    p.write_text(yaml.safe_dump(data))
    return p


def test_valid_route_and_defaults(tmp_path):
    d = load_route(write(tmp_path, valid()))
    assert len(d["goals"]) == 3
    assert d["goal_timeout_wall_seconds"] > 0


@pytest.mark.parametrize(
    "kind",
    [
        "wrong_frame",
        "two_goals",
        "nan_position",
        "infinite_yaw",
        "boolean_position",
        "duplicate_name",
        "near_duplicate_goal",
        "negative_timeout",
        "zero_timeout",
        "bad_initial_pose",
    ],
)
def test_bad_route_rejected_before_motion(tmp_path, kind):
    d = valid()
    if kind == "wrong_frame":
        d["frame_id"] = "odom"
    elif kind == "two_goals":
        d["goals"] = d["goals"][:2]
    elif kind == "nan_position":
        d["goals"][0]["x"] = float("nan")
    elif kind == "infinite_yaw":
        d["goals"][0]["yaw"] = float("inf")
    elif kind == "boolean_position":
        d["goals"][0]["x"] = True
    elif kind == "duplicate_name":
        d["goals"][1]["name"] = d["goals"][0]["name"]
    elif kind == "near_duplicate_goal":
        d["goals"][1]["x"] = 0.2
    elif kind == "negative_timeout":
        d["goal_timeout_wall_seconds"] = -1.0
    elif kind == "zero_timeout":
        d["startup_timeout_wall_seconds"] = 0.0
    elif kind == "bad_initial_pose":
        d["initial_pose"].pop("yaw")
    with pytest.raises(ValueError):
        load_route(write(tmp_path, d))
