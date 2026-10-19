#!/usr/bin/env python3
"""Download missing Ubuntu/ROS packages and extract them inside this project.

Requires Ubuntu 22.04 with ROS 2 Humble Desktop, Nav2 and SLAM Toolbox already
installed. Does not run sudo, dpkg --install, apt install, or maintainer scripts.
The apt simulation supplies exact versions and all missing dependencies.
"""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import re
import shutil
import subprocess


PACKAGES = (
    "ros-humble-turtlebot4-simulator",
    "ros-humble-irobot-create-nodes",
    "ros-humble-turtlebot4-navigation",
    "ros-humble-turtlebot4-viz",
    "ffmpeg",
    "xvfb",
    "xdotool",
)


def run(args: list[str], **kwargs) -> subprocess.CompletedProcess:
    return subprocess.run(args, check=True, text=True, **kwargs)


def deb_identity(path: Path) -> tuple[str, str]:
    result = run(["dpkg-deb", "--field", str(path), "Package", "Version"], capture_output=True)
    fields = dict(line.split(": ", 1) for line in result.stdout.splitlines())
    return fields["Package"], fields["Version"]


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--project", type=Path, default=Path(__file__).resolve().parents[1])
    args = parser.parse_args()
    project = args.project.resolve()
    cache = project / "vendor" / "debs"
    destination = project / ".runtime" / "underlay"
    cache.mkdir(parents=True, exist_ok=True)
    destination.mkdir(parents=True, exist_ok=True)
    simulation = run(
        ["apt-get", "-s", "--no-upgrade", "--no-install-recommends", "install", *PACKAGES],
        capture_output=True,
    )
    requested = re.findall(r"^Inst (\S+)(?: \[[^]]+\])? \((\S+)", simulation.stdout, re.MULTILINE)
    print(f"Preparing {len(requested)} missing packages in {destination}", flush=True)
    available = {deb_identity(path): path for path in cache.glob("*.deb")}
    manifest = []
    for package, version in requested:
        identity = (package, version)
        if identity not in available:
            # Reuse the host's read-only apt cache when an exact archive is there.
            candidates = Path("/var/cache/apt/archives").glob(f"{package}_*.deb")
            for existing in candidates:
                if deb_identity(existing) == identity:
                    archive = cache / existing.name
                    shutil.copyfile(existing, archive)
                    available[identity] = archive
                    break
        if identity not in available:
            print(f"Downloading {package}={version}", flush=True)
            run(["apt-get", "download", f"{package}={version}"], cwd=cache)
            for archive in cache.glob(f"{package}_*.deb"):
                if deb_identity(archive) == identity:
                    available[identity] = archive
                    break
        if identity not in available:
            raise RuntimeError(f"Archive not found for {package}={version}")
        archive = available[identity]
        print(f"Extracting {archive.name}", flush=True)
        run(["dpkg-deb", "--extract", str(archive), str(destination)])
        manifest.append({
            "package": package,
            "version": version,
            "archive": str(archive.relative_to(project)),
            "sha256": hashlib.sha256(archive.read_bytes()).hexdigest(),
        })
    (project / "vendor" / "dependency_manifest.json").write_text(
        json.dumps(manifest, indent=2) + "\n", encoding="utf-8"
    )
    print("Ready. Source scripts/local_underlay.bash before building or running.", flush=True)


if __name__ == "__main__":
    main()
