"""Undock with a temporary backup-limit override, then restore the original mode."""

import argparse
import math
import signal
import time
import uuid

from action_msgs.msg import GoalInfo, GoalStatus
from action_msgs.srv import CancelGoal
from irobot_create_msgs.action import Undock
from irobot_create_msgs.msg import DockStatus
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import GetParameters, SetParameters
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from unique_identifier_msgs.msg import UUID


class UndockRobot(Node):
    def __init__(self, timeout):
        super().__init__('tb4_lab_undock')
        self.timeout = timeout
        self.interrupted = False
        self.dock_status = None
        self.original_mode = None
        self.restore_needed = False
        self.goal_uuid = None
        self.goal_handle = None
        self.result_future = None
        self.action_finished = False
        self.get_client = self.create_client(GetParameters, '/motion_control/get_parameters')
        self.set_client = self.create_client(SetParameters, '/motion_control/set_parameters')
        self.cancel_client = self.create_client(CancelGoal, '/undock/_action/cancel_goal')
        self.undock_client = ActionClient(self, Undock, '/undock')
        self.create_subscription(DockStatus, '/dock_status', self.on_dock_status,
                                 qos_profile_sensor_data)

    def on_dock_status(self, message):
        self.dock_status = message

    def wait_until(self, predicate, deadline, description, interruptible=True):
        while not predicate():
            if interruptible and self.interrupted:
                raise InterruptedError('Interrupted by the operator')
            if not rclpy.ok():
                raise RuntimeError(f'ROS context stopped while waiting for {description}')
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise TimeoutError(f'Timed out waiting for {description}')
            rclpy.spin_once(self, timeout_sec=min(0.1, remaining))
        if interruptible and self.interrupted:
            raise InterruptedError('Interrupted by the operator')

    def wait_future(self, future, deadline, description, interruptible=True):
        self.wait_until(future.done, deadline, description, interruptible)
        result = future.result()
        if result is None:
            raise RuntimeError(f'No response received for {description}')
        return result

    def set_mode(self, mode, deadline, interruptible=True):
        self.wait_until(self.set_client.service_is_ready, deadline,
                        'motion_control/set_parameters', interruptible)
        parameter = Parameter(name='safety_override', value=ParameterValue(
            type=ParameterType.PARAMETER_STRING, string_value=mode))
        future = self.set_client.call_async(SetParameters.Request(parameters=[parameter]))
        response = self.wait_future(future, deadline, f'safety_override={mode}', interruptible)
        if len(response.results) != 1 or not response.results[0].successful:
            reason = response.results[0].reason if response.results else 'empty parameter response'
            raise RuntimeError(f'safety_override={mode} was rejected: {reason}')

    def run(self):
        self.wait_until(lambda: self.dock_status is not None,
                        time.monotonic() + min(10.0, self.timeout), 'dock_status')
        if not self.dock_status.is_docked:
            self.get_logger().info('Observed dock_status.is_docked=false; no undock action needed.')
            return True

        services_deadline = time.monotonic() + min(10.0, self.timeout)
        self.wait_until(self.get_client.service_is_ready, services_deadline,
                        'motion_control/get_parameters')
        self.wait_until(self.set_client.service_is_ready, services_deadline,
                        'motion_control/set_parameters')
        response = self.wait_future(self.get_client.call_async(
            GetParameters.Request(names=['safety_override'])), services_deadline,
            'original safety_override')
        if len(response.values) != 1 or response.values[0].type != ParameterType.PARAMETER_STRING:
            raise RuntimeError('motion_control did not return a string safety_override parameter')
        self.original_mode = response.values[0].string_value
        if self.original_mode not in ['none', 'backup_only', 'full']:
            raise RuntimeError(f'Unrecognized original safety_override: {self.original_mode!r}')
        self.get_logger().info(f'Original safety_override={self.original_mode}.')
        self.wait_until(self.undock_client.server_is_ready, services_deadline, '/undock action server')

        # Mark this before sending: a response timeout may still mean the server applied it.
        self.restore_needed = True
        self.set_mode('backup_only', time.monotonic() + min(10.0, self.timeout))
        self.get_logger().info(
            'Temporary safety_override=backup_only for undocking; reflex parameters unchanged.')
        action_deadline = time.monotonic() + self.timeout
        self.goal_uuid = UUID(uuid=list(uuid.uuid4().bytes))
        self.goal_handle = self.wait_future(self.undock_client.send_goal_async(
            Undock.Goal(), goal_uuid=self.goal_uuid), action_deadline, 'undock goal acceptance')
        if not self.goal_handle.accepted:
            self.action_finished = True
            raise RuntimeError('The /undock action server rejected the goal')
        self.get_logger().info(f'Undock goal accepted: {bytes(self.goal_uuid.uuid).hex()}')
        self.result_future = self.goal_handle.get_result_async()
        result = self.wait_future(self.result_future, action_deadline, 'undock action result')
        self.action_finished = True
        status_names = {GoalStatus.STATUS_SUCCEEDED: 'SUCCEEDED',
                        GoalStatus.STATUS_CANCELED: 'CANCELED',
                        GoalStatus.STATUS_ABORTED: 'ABORTED'}
        name = status_names.get(result.status, str(result.status))
        self.get_logger().info(f'Undock result: status={name}, is_docked={result.result.is_docked}.')
        if result.status != GoalStatus.STATUS_SUCCEEDED or result.result.is_docked:
            raise RuntimeError('Undock did not finish with SUCCEEDED and is_docked=false')
        return True

    def cleanup(self):
        cleanup_ok = True
        if self.goal_uuid is not None and not self.action_finished:
            # Cancel by the known UUID even if the send-goal response timed out.
            self.get_logger().warning('Canceling unfinished undock goal before restoring safety mode.')
            try:
                deadline = time.monotonic() + 5.0
                self.wait_until(self.cancel_client.service_is_ready, deadline,
                                'undock cancellation service', interruptible=False)
                response = self.wait_future(self.cancel_client.call_async(CancelGoal.Request(
                    goal_info=GoalInfo(goal_id=self.goal_uuid))), deadline,
                    'undock cancellation response', interruptible=False)
                if response.return_code not in [CancelGoal.Response.ERROR_NONE,
                                                CancelGoal.Response.ERROR_GOAL_TERMINATED]:
                    raise RuntimeError(f'Cancellation returned code {response.return_code}')
                if self.result_future is not None:
                    self.wait_future(self.result_future, time.monotonic() + 5.0,
                                     'terminal undock state after cancellation', interruptible=False)
                self.get_logger().info('Undock cancellation acknowledged or goal already terminal.')
            except Exception as error:
                cleanup_ok = False
                self.get_logger().error(f'Could not confirm undock cancellation: {error}')
        if self.restore_needed:
            try:
                self.set_mode(self.original_mode, time.monotonic() + 10.0, interruptible=False)
                self.get_logger().info(f'Restored safety_override={self.original_mode}.')
            except Exception as error:
                cleanup_ok = False
                self.get_logger().error(
                    f'Could not restore safety_override={self.original_mode}: {error}. '
                    f'Restore it with ros2 param set /motion_control safety_override {self.original_mode}')
        return cleanup_ok


def main(args=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--timeout', type=float, default=30.0,
                        help='Maximum wall seconds for the undock action (default: 30)')
    options, ros_args = parser.parse_known_args(args)
    if options.timeout <= 0 or not math.isfinite(options.timeout):
        parser.error('--timeout must be finite and positive')
    # Keep the context alive on SIGINT/SIGTERM so the finally block can restore the parameter.
    rclpy.init(args=ros_args, signal_handler_options=SignalHandlerOptions.NO)
    node = UndockRobot(options.timeout)
    old_handlers = {}

    def interrupt_handler(_signum, _frame):
        node.interrupted = True

    for signum in [signal.SIGINT, signal.SIGTERM]:
        old_handlers[signum] = signal.signal(signum, interrupt_handler)
    successful = False
    try:
        successful = node.run()
    except (RuntimeError, TimeoutError, InterruptedError) as error:
        node.get_logger().error(str(error))
    finally:
        successful = node.cleanup() and successful
        node.undock_client.destroy()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        for signum, handler in old_handlers.items():
            signal.signal(signum, handler)
    raise SystemExit(0 if successful else 1)


if __name__ == '__main__':
    main()
