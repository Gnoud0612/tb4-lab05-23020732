#!/usr/bin/env python3
"""Audit real mission/action/camera logs; never generate missing successes."""
from pathlib import Path
import hashlib, json, math

root = Path(__file__).resolve().parents[1]
mission = json.loads((root / "evidence/mission.json").read_text())
monitor = json.loads((root / "evidence/navigation_goals.json").read_text())
provenance = json.loads((root / "maps/provenance_lab04.json").read_text())
failures = []
for name, expected in provenance["files"].items():
    if hashlib.sha256((root / "maps" / name).read_bytes()).hexdigest() != expected:
        failures.append("Lab04 map changed: " + name)
goals = mission["goals"]
if (
    not mission["passed"]
    or len(goals) < 3
    or any(g["terminal_status"] != "SUCCEEDED" for g in goals)
):
    failures.append("Mission must finish with all sequential goals SUCCEEDED")
if [g["index"] for g in goals] != list(range(1, len(goals) + 1)):
    failures.append("Goal sequence is inconsistent")
if not monitor["summary"]["monitor_criteria_met"]:
    failures.append("Independent goal monitor did not pass")
observed = {g["goal_id"]: g for g in monitor["goals"]}
for g in goals:
    m = observed.get(g.get("goal_id"))
    if not m or m["terminal_status"] != "SUCCEEDED":
        failures.append("UUID not independently confirmed")
    if g.get("bump_detected") or g.get("hazard_messages", 0) == 0:
        failures.append("BUMP or missing hazard observations")
events = mission["events"]
camera = []
for line in (root / "evidence/yolo/detections.jsonl").read_text().splitlines():
    d = json.loads(line)
    if (
        d["source_topic"] != "/oakd/rgb/preview/image_raw"
        or d["frame_id"] != "oakd_rgb_camera_optical_frame"
    ):
        failures.append("Unexpected camera source")
    camera.append(d)
per_goal = []
classes = set()
for g in goals:
    start = [
        e for e in events if e["event"] == "goal_sent" and e["index"] == g["index"]
    ]
    end = [
        e for e in events if e["event"] == "goal_result" and e["index"] == g["index"]
    ]
    if not start or not end:
        failures.append("Missing action times")
        continue
    frames = [
        d
        for d in camera
        if start[0]["simulation_time_seconds"]
        <= d["image_stamp"]
        <= end[0]["simulation_time_seconds"]
    ]
    if not frames:
        failures.append("No live YOLO camera frame during goal " + str(g["index"]))
    det = [v for d in frames for v in d["detections"]]
    for v in det:
        if (
            not 0 <= v["confidence"] <= 1
            or len(v["xyxy"]) != 4
            or not all(math.isfinite(x) for x in v["xyxy"])
        ):
            failures.append("Invalid model detection values")
        classes.add(v["class_name"])
    per_goal.append(
        {
            "goal": g["index"],
            "camera_frames_during_goal": len(frames),
            "detections": len(det),
            "classes": sorted({v["class_name"] for v in det}),
        }
    )
if not classes:
    failures.append("No actual object detections during navigation")
result = {
    "passed": not failures,
    "failures": failures,
    "map_unchanged": not any("map changed" in s for s in failures),
    "successful_sequential_goals": sum(
        g["terminal_status"] == "SUCCEEDED" for g in goals
    ),
    "classes_detected_during_navigation": sorted(classes),
    "camera_per_goal": per_goal,
    "limitations": [
        "No BUMP messages is sampled evidence; review the real video too.",
        "Object detection confidence is not a calibrated accuracy score.",
    ],
}
(root / "evidence/validation.json").write_text(json.dumps(result, indent=2) + "\n")
print(json.dumps(result, indent=2))
raise SystemExit(0 if result["passed"] else 1)
