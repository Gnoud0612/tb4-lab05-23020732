"""Apply the flat-maze simulation profile without moving the robot."""

import argparse
from datetime import datetime, timezone
import json
import math
import os
from pathlib import Path
import time

from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import GetParameters, SetParametersAtomically
import rclpy
from rclpy.node import Node


SAFETY_PARAMETER = 'safety_override'
CLIFF_PARAMETER = 'reflexes.REFLEX_CLIFF'
PRESERVED_REFLEXES = (
    'reflexes_enabled',
    'reflexes.REFLEX_BUMP',
    'reflexes.REFLEX_STUCK',
    'reflexes.REFLEX_WHEEL_DROP',
)
PARAMETER_NAMES = (SAFETY_PARAMETER, CLIFF_PARAMETER) + PRESERVED_REFLEXES


class SimulationProfile(Node):
    def __init__(self):
        super().__init__('tb4_lab_sim_profile')
        self.get_client = self.create_client(GetParameters, '/motion_control/get_parameters')
        self.atomic_client = self.create_client(
            SetParametersAtomically, '/motion_control/set_parameters_atomically')

    def wait_until(self, predicate, deadline, description):
        while not predicate():
            if not rclpy.ok():
                raise RuntimeError(f'ROS context stopped while waiting for {description}')
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise TimeoutError(f'Timed out waiting for {description}')
            rclpy.spin_once(self, timeout_sec=min(0.1, remaining))

    def wait_future(self, future, deadline, description):
        self.wait_until(future.done, deadline, description)
        response = future.result()
        if response is None:
            raise RuntimeError(f'No response for {description}')
        return response

    def read_parameters(self, deadline):
        response = self.wait_future(self.get_client.call_async(
            GetParameters.Request(names=list(PARAMETER_NAMES))), deadline,
            'motion_control parameter values')
        if len(response.values) != len(PARAMETER_NAMES):
            raise RuntimeError('motion_control returned an incomplete parameter response')
        values = {}
        for name, value in zip(PARAMETER_NAMES, response.values):
            expected_type = (ParameterType.PARAMETER_STRING if name == SAFETY_PARAMETER
                             else ParameterType.PARAMETER_BOOL)
            if value.type != expected_type:
                raise RuntimeError(f'{name} has unexpected type {value.type}')
            values[name] = (value.string_value if name == SAFETY_PARAMETER
                            else value.bool_value)
        if values[SAFETY_PARAMETER] not in ('none', 'backup_only', 'full'):
            raise RuntimeError(f'Unrecognized safety_override: {values[SAFETY_PARAMETER]!r}')
        return values

    def apply(self, timeout, report):
        deadline = time.monotonic() + timeout
        self.wait_until(self.get_client.service_is_ready, deadline,
                        'motion_control/get_parameters')
        self.wait_until(self.atomic_client.service_is_ready, deadline,
                        'motion_control/set_parameters_atomically')
        before = self.read_parameters(deadline)
        report['parameters_before'] = before
        disabled = [name for name in PRESERVED_REFLEXES if not before[name]]
        if disabled:
            raise RuntimeError('Required reflexes are already disabled: ' + ', '.join(disabled))
        mode = before[SAFETY_PARAMETER]
        # irobot_create_nodes 2.1.0 has separate motion-control/reflex callbacks.
        # Pairing the unchanged safety mode allows both callbacks to accept this request.
        requested = {SAFETY_PARAMETER: mode, CLIFF_PARAMETER: False}
        report['atomic_request'] = requested
        request = SetParametersAtomically.Request(parameters=[
            Parameter(name=SAFETY_PARAMETER, value=ParameterValue(
                type=ParameterType.PARAMETER_STRING, string_value=mode)),
            Parameter(name=CLIFF_PARAMETER, value=ParameterValue(
                type=ParameterType.PARAMETER_BOOL, bool_value=False)),
        ])
        report['update_sent'] = True
        response = self.wait_future(self.atomic_client.call_async(request), deadline,
                                    'atomic simulation profile update')
        report['atomic_result'] = {
            'successful': response.result.successful,
            'reason': response.result.reason,
        }
        if not response.result.successful:
            raise RuntimeError('Simulation profile was rejected: ' + response.result.reason)
        after = self.read_parameters(deadline)
        report['parameters_after'] = after
        checks = {
            'cliff_reflex_disabled': after[CLIFF_PARAMETER] is False,
            'safety_override_preserved': after[SAFETY_PARAMETER] == mode,
            'other_reflexes_preserved': all(after[name] == before[name]
                                           for name in PRESERVED_REFLEXES),
            'other_reflexes_enabled': all(after[name] for name in PRESERVED_REFLEXES),
        }
        report['readback_checks'] = checks
        if not all(checks.values()):
            raise RuntimeError('Simulation profile readback failed: ' + ', '.join(
                name for name, passed in checks.items() if not passed))
        self.get_logger().info(
            f'Flat-maze profile verified: CLIFF reflex=false; safety_override={mode} preserved; '
            'BUMP, STUCK and WHEEL_DROP reflexes remain enabled.')


def save_report(path, report):
    path = Path(path).expanduser().resolve()
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(path.name + '.tmp')
    temporary.write_text(json.dumps(report, ensure_ascii=False, indent=2) + '\n',
                         encoding='utf-8')
    temporary.replace(path)
    return path


def main(args=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--world', required=True, choices=['maze'],
                        help='Explicitly confirm the official flat maze world')
    parser.add_argument('--timeout', type=float, default=15.0,
                        help='Maximum wall seconds for service discovery/update/readback (default: 15)')
    workspace = os.environ.get('TB4_WORKSPACE', '.')
    parser.add_argument('--output', default=str(Path(workspace) / 'evidence' / 'sim_profile.json'),
                        help='JSON verification report')
    options, ros_args = parser.parse_known_args(args)
    if not math.isfinite(options.timeout) or options.timeout <= 0:
        parser.error('--timeout must be finite and positive')
    report = {
        'world': options.world,
        'flat_world_only': True,
        'world_confirmation': 'Operator-supplied --world maze; world geometry is not inspected by this tool.',
        'reason': {
            'prior_measured_issue': 'CLIFF hazards occurred on the flat official maze floor. '
                                    'front_right and side_left cliff scans had no finite ray return.',
            'source_detection_threshold_m': 0.03,
            'source': 'irobot_create_ignition_toolbox/src/sensors/cliff.cpp, create3_sim tag 2.1.0',
            'automatic_sensor_remeasurement': False,
            'rendering_cause': 'Not established by the measurements.',
        },
        'limitations': 'This profile applies only to the flat maze simulation. '
                       'Hazard publications and other reflexes remain active. '
                       'It does not command motion or cancel an already-running reflex.',
        'started_at_utc': datetime.now(timezone.utc).isoformat(),
        'timeout_wall_seconds': options.timeout,
        'update_sent': False,
        'passed': False,
        'errors': [],
    }
    started = time.monotonic()
    node = None
    try:
        rclpy.init(args=ros_args)
        node = SimulationProfile()
        node.apply(options.timeout, report)
        report['passed'] = True
    except (Exception, KeyboardInterrupt) as error:
        report['errors'].append(str(error) or type(error).__name__)
        if node is not None:
            node.get_logger().error(report['errors'][-1])
    finally:
        report['finished_at_utc'] = datetime.now(timezone.utc).isoformat()
        report['elapsed_wall_seconds'] = time.monotonic() - started
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        try:
            path = save_report(options.output, report)
            print(f'Simulation profile evidence: {path}; passed={report["passed"]}')
        except OSError as error:
            report['passed'] = False
            print(f'Could not save simulation profile evidence: {error}')
    raise SystemExit(0 if report['passed'] else 1)


if __name__ == '__main__':
    main()
