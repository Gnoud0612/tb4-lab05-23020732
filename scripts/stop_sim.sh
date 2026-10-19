#!/usr/bin/env bash
# Stop only the project partition and descendants captured from its verified
# registered launch PID. Each signal checks PID + Linux process start time.
set -e
source "$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)/env.bash"
python3 - "${TB4_WORKSPACE}" <<'PY'
import os
import json
from pathlib import Path
import signal
import subprocess
import sys
import time

project = Path(sys.argv[1]).resolve()
partition = os.environ['IGN_PARTITION']
if partition != f"tb4_lab_domain{os.environ['ROS_DOMAIN_ID']}" or os.environ.get('GZ_PARTITION') != partition:
    sys.exit('Refusing to stop a simulator outside the expected project partition.')

pid_file = project / '.runtime/launch.pid'
registry_file = project / '.runtime/launch_processes.json'
registration = pid_file.read_text().strip() if pid_file.exists() else None
try:
    registered_pid = int(registration) if registration is not None else None
except ValueError:
    sys.exit(f'Invalid registered launch PID in {pid_file}.')
if registered_pid is not None and registered_pid <= 1:
    sys.exit('Refusing an invalid launch PID.')

def identity(pid):
    """Linux start ticks reject reused PIDs; zombies are already stopped."""
    try:
        fields = Path(f'/proc/{pid}/stat').read_text().rpartition(')')[2].split()
        return {'pid': pid, 'ppid': int(fields[1]),
                'start_ticks': int(fields[19]), 'state': fields[0]}
    except (OSError, IndexError, ValueError):
        return None

def alive(record):
    current = identity(record['pid'])
    return current is not None and current['state'] != 'Z' and current['start_ticks'] == record['start_ticks']

def guard_registration():
    current = pid_file.read_text().strip() if pid_file.exists() else None
    if current != registration:
        sys.exit('Launch registration changed while stopping; refusing further shutdown actions.')

def descendants(root_record):
    if not alive(root_record):
        return []
    table = {}
    for entry in Path('/proc').iterdir():
        if entry.name.isdigit():
            record = identity(int(entry.name))
            if record is not None:
                table[record['pid']] = record
    included = {root_record['pid']}
    while True:
        children = {pid for pid, record in table.items() if record['ppid'] in included}
        if children <= included:
            break
        included.update(children)
    return [table[pid] for pid in sorted(included) if pid in table]

root = identity(registered_pid) if registered_pid is not None else None
captured = {}
if root is not None and root['state'] != 'Z':
    process = Path(f'/proc/{registered_pid}')
    command = process.joinpath('cmdline').read_bytes().decode().split('\0')
    is_lab_launch = any(Path(command[i]).name == 'ros2' and command[i+1:i+3] == ['launch', 'tb4_lab']
                        for i in range(max(0, len(command)-2)))
    environment = dict(item.split('=', 1) for item in process.joinpath('environ').read_bytes().decode().split('\0') if '=' in item)
    if not is_lab_launch or environment.get('IGN_PARTITION') != partition:
        sys.exit(f'Refusing PID {registered_pid}: it is not a tb4_lab launch in {partition}.')
    captured = {r['pid']: r for r in descendants(root)}

# A terminal may disappear before cleanup and reparent ROS children. Recover
# only ROS executables bearing all three exact project environment markers.
# RViz, observers and unrelated ROS workspaces are excluded.
owned_prefixes = ('/opt/ros/humble/lib/', str(project / '.runtime/underlay/opt/ros/humble/lib') + '/')
recovered = 0
for entry in Path('/proc').iterdir():
    if not entry.name.isdigit():
        continue
    try:
        env = dict(item.split('=', 1) for item in entry.joinpath('environ').read_bytes().decode().split('\0') if '=' in item)
        args = entry.joinpath('cmdline').read_bytes().decode().split('\0')
        owned = (env.get('TB4_WORKSPACE') == str(project)
                 and env.get('IGN_PARTITION') == partition
                 and env.get('ROS_LOG_DIR') == str(project / '.runtime/ros_logs'))
        ros_node = '--ros-args' in args and any(arg.startswith(owned_prefixes) for arg in args)
        record = identity(int(entry.name))
        if owned and ros_node and record and record['state'] != 'Z':
            if record['pid'] not in captured:
                recovered += 1
            captured[record['pid']] = record
    except (OSError, UnicodeError, ValueError):
        continue
print(f'Recovered {recovered} ROS process identities by exact project environment markers.', flush=True)

# A previous interrupted stop may have recorded children which became orphans.
# Only recover identities previously captured from this exact project/launch.
if registry_file.exists():
    previous = json.loads(registry_file.read_text())
    if previous.get('project') == str(project) and previous.get('partition') == partition and previous.get('launch_pid') == registered_pid:
        for record in previous.get('processes', []):
            if alive(record):
                captured.setdefault(record['pid'], record)

def save_registry():
    guard_registration()
    document = {'project': str(project), 'partition': partition,
                'launch_pid': registered_pid, 'processes': list(captured.values())}
    temporary = registry_file.with_suffix('.json.tmp')
    temporary.write_text(json.dumps(document, indent=2)+'\n')
    temporary.replace(registry_file)

save_registry()
print(f'Captured {len(captured)} launch process identities before shutdown.', flush=True)
guard_registration()
print(f'Requesting Gazebo shutdown only in partition {partition}...', flush=True)
try:
    response = subprocess.run([
        'ign', 'service', '-s', '/server_control',
        '--reqtype', 'ignition.msgs.ServerControl', '--reptype', 'ignition.msgs.Boolean',
        '--timeout', '3000', '--req', 'stop: true',
    ], text=True, capture_output=True, timeout=6)
    if response.stdout.strip(): print(response.stdout.strip())
    if response.stderr.strip(): print(response.stderr.strip(), file=sys.stderr)
except subprocess.TimeoutExpired:
    print('Gazebo service timed out; continuing only with captured launch identities.', file=sys.stderr)

# Capture any child spawned during the service call while the same parent lives.
if root is not None:
    for record in descendants(root):
        captured.setdefault(record['pid'], record)
save_registry()

def signal_record(record, signum):
    guard_registration()
    if not alive(record): return
    try:
        # A pidfd also closes the tiny race between checking /proc and signaling
        # a PID that could be recycled. Ubuntu 22.04 supports this API.
        if hasattr(os, 'pidfd_open') and hasattr(signal, 'pidfd_send_signal'):
            descriptor = os.pidfd_open(record['pid'])
            try:
                if not alive(record): return
                signal.pidfd_send_signal(descriptor, signum)
            finally:
                os.close(descriptor)
        else:
            os.kill(record['pid'], signum)
        print(f'Sent {signal.Signals(signum).name} to captured PID {record["pid"]}.', flush=True)
    except ProcessLookupError:
        pass

if root is not None:
    signal_record(root, signal.SIGINT)

def wait_for_exit(seconds):
    deadline = time.monotonic()+seconds
    while any(alive(record) for record in captured.values()) and time.monotonic() < deadline:
        guard_registration()
        time.sleep(0.2)

wait_for_exit(10.0)
remaining = [record for record in captured.values() if alive(record)]
if remaining:
    print('Captured descendants remain; sending SIGTERM only to their matching identities.', flush=True)
    for record in reversed(remaining): signal_record(record, signal.SIGTERM)
    wait_for_exit(5.0)
remaining = [record for record in captured.values() if alive(record)]
if remaining:
    print('Captured descendants still remain; sending SIGKILL only to their matching identities.', flush=True)
    for record in reversed(remaining): signal_record(record, signal.SIGKILL)
    wait_for_exit(2.0)
remaining = [record['pid'] for record in captured.values() if alive(record)]
if remaining:
    sys.exit(f'Captured processes still exist: {remaining}; registration retained for recovery.')
guard_registration()
if pid_file.exists(): pid_file.unlink()
if registry_file.exists(): registry_file.unlink()
print('All captured project launch processes have exited. Unrelated processes were not signaled.')
PY
