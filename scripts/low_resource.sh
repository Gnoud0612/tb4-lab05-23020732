#!/usr/bin/env bash
# Official Gazebo sensor services adjust rates for software-rendered machines.
set -e
source "$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)/env.bash"
python3 - "${TB4_WORKSPACE}" "$@" <<'PY'
import argparse
from collections import Counter
from concurrent.futures import ThreadPoolExecutor
from datetime import datetime, timezone
import json
from pathlib import Path
import re
import subprocess
import sys
import time

import yaml

parser = argparse.ArgumentParser(description='Reduce sensor rendering workload through Gazebo services.')
parser.add_argument('project', type=Path)
parser.add_argument('--startup-timeout', type=float, default=60.0,
                    help='Maximum wall seconds to wait for all 17 robot and dock services.')
parser.add_argument('--verify-seconds', type=float, default=8.0,
                    help='Wall seconds to sample /scan timestamps after requests.')
args = parser.parse_args()
if args.startup_timeout <= 0 or args.verify_seconds <= 0:
    parser.error('Both startup timeout and verification duration must be positive.')
project = args.project
profile_file = project / 'src/tb4_lab/config/sensor_rates.yaml'
profiles = yaml.safe_load(profile_file.read_text())['profiles']
requests = []
startup_deadline = time.monotonic() + args.startup_timeout
expected = {p['name']: int(p['expected_count']) for p in profiles}
found = Counter()
print(f'Waiting up to {args.startup_timeout:g} seconds for all {sum(expected.values())} sensor services...', flush=True)
while time.monotonic() < startup_deadline:
    remaining = startup_deadline - time.monotonic()
    try:
        listed = subprocess.run(['ign', 'service', '-l'], check=True, text=True,
                                capture_output=True, timeout=min(5.0, remaining))
        services = set(listed.stdout.splitlines())
    except (subprocess.TimeoutExpired, subprocess.CalledProcessError):
        services = set()
    requests = []
    for service in sorted(services):
        for profile in profiles:
            if re.search(profile['service_pattern'], service):
                requests.append({'service': service, 'profile': profile['name'],
                                 'rate_hz': float(profile['rate_hz'])})
                break
    found = Counter(r['profile'] for r in requests)
    if all(found[name] == count for name, count in expected.items()):
        break
    print(f'Sensor services found: {sum(found.values())}/{sum(expected.values())}. Waiting...', flush=True)
    time.sleep(min(0.5, max(0.0, startup_deadline-time.monotonic())))
else:
    sys.exit(f'Sensor startup timed out after {args.startup_timeout:g} seconds. '
             f'Expected {expected}, found {dict(found)}. No rates were changed.')

def request_rate(request):
    result = subprocess.run([
        'ign', 'service', '-s', request['service'],
        '--reqtype', 'ignition.msgs.Double', '--reptype', 'ignition.msgs.Empty',
        '--timeout', '500', '--req', f"data: {request['rate_hz']}",
    ], text=True, capture_output=True, timeout=10)
    # SensorPrivate::SetRate is a void callback, so it does not send an ACK.
    # Fortress CLI still waits for a response and prints a timeout after delivery.
    # This output is kept verbatim and never represented as an acknowledged call.
    request.update({'cli_returncode': result.returncode,
                    'cli_stdout': result.stdout.strip(),
                    'cli_stderr': result.stderr.strip(),
                    'response_expected': False})
    print(f"Requested {request['profile']}: {request['rate_hz']:g} Hz at {request['service']}", flush=True)
    return request

with ThreadPoolExecutor(max_workers=4) as executor:
    results = list(executor.map(request_rate, requests))

verification = {'topic': '/scan', 'wall_sample_seconds': args.verify_seconds,
                'note': 'Service has no response; topic timestamps verify LiDAR frequency independently.'}
verification_passed = False
if args.verify_seconds > 0:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import LaserScan

    rclpy.init()
    node = Node('tb4_sensor_rate_verification')
    stamps = []
    def sample(msg):
        stamps.append(msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9)
    subscription = node.create_subscription(LaserScan, '/scan', sample, qos_profile_sensor_data)
    deadline = time.monotonic() + args.verify_seconds
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=min(0.2, max(0.0, deadline-time.monotonic())))
    unique = sorted(set(stamps))
    verification['received_messages'] = len(stamps)
    verification['unique_timestamps'] = len(unique)
    if len(unique) >= 5:
        duration = unique[-1] - unique[0]
        frequency = (len(unique)-1)/duration
        verification.update({'sim_timestamp_span_seconds': duration,
                             'observed_sim_frequency_hz': frequency})
        verification_passed = 8.0 <= frequency <= 12.0
        print(f'LiDAR observed {frequency:.2f} Hz by simulation timestamps.')
    else:
        print('Insufficient LiDAR samples: at least 5 unique timestamps are required.')
    node.destroy_node()
    rclpy.shutdown()
verification['passed'] = verification_passed
output = project / '.runtime/sensor_rates_result.json'
output.write_text(json.dumps({'created_utc': datetime.now(timezone.utc).isoformat(),
                             'profile': str(profile_file.relative_to(project)),
                             'requests': results, 'verification': verification}, indent=2)+'\n')
print(f'Raw service outputs and independent frequency observation saved to {output}.')
if not verification_passed:
    sys.exit('LiDAR rate verification failed: require at least 5 timestamps at 8..12 Hz. '
             'Do not undock or drive until scripts/low_resource.sh exits successfully.')
PY
