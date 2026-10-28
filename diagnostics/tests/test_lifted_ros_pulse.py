"""Exercise the pulse with installed ROS imports but no ROS context/devices."""
import contextlib
import importlib.util
import io
import math
from pathlib import Path
import sys
from types import SimpleNamespace as NS
import unittest
from unittest.mock import patch

import rclpy
from nav_msgs.msg import Odometry
from rcl_interfaces.srv import GetParameters


SOURCE = Path(__file__).resolve().parents[1] / 'leader_lifted_ros_pulse.py'
SPEC = importlib.util.spec_from_file_location('pulse_under_test', SOURCE)
pulse = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(pulse)


class Clock:
    def __init__(self):
        self.t = 100.0

    def monotonic(self):
        return self.t


class FakeNode:
    def __init__(self, clock, mode):
        self.clock, self.mode = clock, mode
        self.callbacks, self.sent = {}, []
        self.speed, self.x = 0.0, 0.0
        self.parameter_request_seen = False

    def create_client(self, service_type, name):
        assert service_type is GetParameters
        assert name == '/md200t_controller/get_parameters'
        return NS(wait_for_service=lambda **kw: self.mode != 'no_service',
                  call_async=self.get_parameters)

    def get_parameters(self, request):
        assert isinstance(request, GetParameters.Request)
        assert request.names == ['allow_motion', 'max_rpm', 'dry_run']
        self.parameter_request_seen = True
        response = NS(values=[NS(bool_value=True),
                             NS(integer_value=100 if self.mode == 'wrong_limit' else 5),
                             NS(bool_value=False)])
        return NS(done=lambda: True, result=lambda: response)

    def create_publisher(self, *args):
        return NS(publish=self.publish)

    def publish(self, msg):
        self.speed = msg.linear.x
        self.sent.append((self.clock.t, msg.linear.x, msg.angular.z))

    def create_subscription(self, msg_type, topic, callback, depth):
        self.callbacks[topic] = callback

    def get_name(self):
        return 'leader_lifted_pulse_check'

    def get_publishers_info_by_topic(self, topic):
        return [NS(node_name='other' if self.mode == 'other_publisher' else self.get_name(),
                   node_namespace='/')]

    def get_subscriptions_info_by_topic(self, topic):
        subscribers = [NS(node_name='md200t_controller', node_namespace='/')]
        if self.mode in ('recorder', 'unexpected_subscriber'):
            subscribers.append(NS(
                node_name='rosbag2_recorder' if self.mode == 'recorder' else 'other_controller',
                node_namespace='/'))
        return subscribers

    def get_clock(self):
        return NS(now=lambda: NS(nanoseconds=int(self.clock.t * 1e9)))

    def spin(self, timeout_sec):
        dt = max(timeout_sec, 0.001)
        self.clock.t += dt
        speed = 0.0 if self.mode == 'stalled' else min(
            self.speed, 5 * 2 * math.pi * 0.085 / 60)
        self.x += speed * dt
        for callback in self.callbacks.values():
            msg = Odometry()
            stamp = self.clock.t - (2 if self.mode == 'stale' else 0)
            ns = int(stamp * 1e9)
            msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(ns, 10**9)
            msg.pose.pose.position.x = self.x
            if self.mode == 'pose_jump' and self.speed > 0:
                msg.pose.pose.position.x += 1.0
            if self.mode == 'large_uncertainty':
                msg.pose.covariance[7] = 1000.0
            if self.mode == 'nonfinite':
                msg.pose.pose.position.x = float('nan')
            msg.twist.twist.linear.x = speed
            callback(msg)

    def destroy_node(self):
        pass


class PulseTest(unittest.TestCase):
    def run_case(self, mode):
        clock = Clock()
        node = FakeNode(clock, mode)
        with contextlib.ExitStack() as stack:
            stack.enter_context(patch.object(pulse, 'time', clock))
            stack.enter_context(patch.object(pulse.signal, 'signal', lambda *a: None))
            stack.enter_context(patch.object(sys, 'argv', [str(SOURCE), '--run-lifted-test']))
            for name, replacement in {
                'init': lambda **kw: None,
                'create_node': lambda name: node,
                'ok': lambda: True,
                'shutdown': lambda: None,
                'spin_once': lambda n, timeout_sec: node.spin(timeout_sec),
                'spin_until_future_complete': lambda *a, **kw: None,
            }.items():
                stack.enter_context(patch.object(rclpy, name, replacement))
            stack.enter_context(contextlib.redirect_stdout(io.StringIO()))
            result = pulse.main()
        self.assertEqual(node.sent[-1][1:], (0.0, 0.0))
        self.assertTrue(all(v in (0.0, 0.04) and w == 0.0 for _, v, w in node.sent))
        moving = [t for t, v, _ in node.sent if v]
        if mode in ('normal', 'recorder'):
            self.assertEqual(result, 0)
            self.assertTrue(node.parameter_request_seen)
            self.assertTrue(moving)
            first_stop = next(t for t, v, _ in node.sent if t > moving[0] and v == 0)
            self.assertLess(first_stop - moving[0], 1.6)
        else:
            self.assertEqual(result, 2)
            if mode not in ('stalled', 'pose_jump'):
                self.assertFalse(moving)

    def test_installed_humble_service_api_and_normal_pulse(self):
        self.run_case('normal')

    def test_wrong_rpm_limit_blocks_motion(self):
        self.run_case('wrong_limit')

    def test_other_publisher_blocks_motion(self):
        self.run_case('other_publisher')

    def test_stale_odometry_blocks_motion(self):
        self.run_case('stale')

    def test_missing_parameter_service_blocks_motion(self):
        self.run_case('no_service')

    def test_absent_movement_is_not_reported_as_success(self):
        self.run_case('stalled')

    def test_rosbag_observer_is_allowed(self):
        self.run_case('recorder')

    def test_unexpected_command_subscriber_blocks_motion(self):
        self.run_case('unexpected_subscriber')

    def test_position_jump_interrupts_pulse(self):
        self.run_case('pose_jump')

    def test_large_uncertainty_blocks_motion(self):
        self.run_case('large_uncertainty')

    def test_nonfinite_position_blocks_motion(self):
        self.run_case('nonfinite')


if __name__ == '__main__':
    unittest.main()
