#!/usr/bin/env python3
"""Spawn real OSRF 3D models via Gazebo's official EntityFactory service."""
import json, math, os, re, subprocess, time
from pathlib import Path
import xml.etree.ElementTree as ET
import yaml

root = Path(__file__).resolve().parents[1]
config = yaml.safe_load((root / "src/tb4_yolo_nav/config/objects.yaml").read_text())
world = config["world"]
service = f"/world/{world}/create"
deadline = time.monotonic() + 60
while time.monotonic() < deadline:
    result = subprocess.run(
        ["ign", "service", "-l"], text=True, capture_output=True, timeout=8
    )
    if service in result.stdout.splitlines():
        break
    time.sleep(0.5)
else:
    raise SystemExit("Gazebo create service not found")
out = root / ".runtime/objects_sdf"
out.mkdir(parents=True, exist_ok=True)
records = []
for obj in config["objects"]:
    source = root / "models" / obj["model"] / "model.sdf"
    tree = ET.parse(source)
    model = tree.getroot().find("model")
    model.set("name", obj["name"])
    static = model.find("static")
    if static is None:
        static = ET.SubElement(model, "static")
    static.text = "true"
    for uri in model.findall(".//uri"):
        if uri.text.startswith("model://"):
            path = root / "models" / uri.text.removeprefix("model://")
            if not path.is_file():
                raise FileNotFoundError(path)
            uri.text = str(path.resolve())
    path = out / (obj["name"] + ".sdf")
    tree.write(path, encoding="unicode", xml_declaration=True)
    x, y, z, yaw = [float(obj[k]) for k in ["x", "y", "z", "yaw"]]
    req = f'sdf_filename: "{path}" name: "{obj["name"]}" allow_renaming: false pose {{ position {{ x: {x} y: {y} z: {z} }} orientation {{ z: {math.sin(yaw/2)} w: {math.cos(yaw/2)} }} }}'
    result = subprocess.run(
        [
            "ign",
            "service",
            "-s",
            service,
            "--reqtype",
            "ignition.msgs.EntityFactory",
            "--reptype",
            "ignition.msgs.Boolean",
            "--timeout",
            "10000",
            "--req",
            req,
        ],
        text=True,
        capture_output=True,
        timeout=15,
    )
    accepted = result.returncode == 0 and "data: true" in result.stdout
    records.append(
        {
            "object": obj,
            "accepted": accepted,
            "stdout": result.stdout,
            "stderr": result.stderr,
        }
    )
    print(obj["name"], "SPAWNED" if accepted else "FAILED", flush=True)
    if not accepted:
        (root / "evidence/objects_spawn.json").write_text(
            json.dumps(records, indent=2) + "\n"
        )
        raise SystemExit(result.stdout + result.stderr)
(root / "evidence/objects_spawn.json").write_text(json.dumps(records, indent=2) + "\n")
