"""Simulated ground-check regression tests. No ROS context or hardware."""
import contextlib
import importlib.util
import io
import math
from pathlib import Path
from types import SimpleNamespace as NS
import unittest
from unittest.mock import patch

import rclpy
from nav_msgs.msg import Odometry
from rcl_interfaces.srv import GetParameters


SOURCE = Path(__file__).resolve().parents[1] / 'leader_ground_straight_check.py'
SPEC = importlib.util.spec_from_file_location('ground_under_test', SOURCE)
ground = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(ground)


class Clock:
    def __init__(self):
        self.t = 100.0

    def monotonic(self):
        return self.t


class FakeNode:
    def __init__(self, clock, mode):
        self.clock, self.mode = clock, mode
        self.callbacks, self.sent = {}, []
        self.speed, self.x, self.ever_moved = 0.0, 0.0, False

    def create_publisher(self, *args):
        return NS(publish=self.publish)

    def create_subscription(self, msg_type, topic, callback, depth):
        self.callbacks[topic] = callback

    def publish(self, msg):
        self.sent.append((self.clock.t, msg.linear.x, msg.angular.z))
        if not (self.mode == 'stop_failure' and self.ever_moved and msg.linear.x == 0):
            self.speed = msg.linear.x
        self.ever_moved = self.ever_moved or msg.linear.x > 0

    def create_client(self, cls, name):
        assert cls is GetParameters
        def call(request):
            if name == '/md200t_controller/get_parameters':
                values = [NS(bool_value=True), NS(integer_value=100 if self.mode == 'wrong_limit' else 5),
                          NS(bool_value=False), NS(bool_value=True), NS(double_value=10.0)]
            else:
                selection = [False]*15
                selection[6] = selection[7] = True
                values = [NS(bool_array_value=selection), NS(string_value='odom'), NS(bool_value=False)]
            assert len(values) == len(request.names)
            return NS(done=lambda: True, result=lambda: NS(values=values))
        return NS(wait_for_service=lambda **kw: True, call_async=call)

    def get_name(self):
        return 'leader_ground_straight_check'

    def get_publishers_info_by_topic(self, topic):
        return [NS(node_name='other' if self.mode == 'other_publisher' else self.get_name(),
                   node_namespace='/')]

    def get_subscriptions_info_by_topic(self, topic):
        return [NS(node_name='md200t_controller', node_namespace='/')]

    def get_clock(self):
        return NS(now=lambda: NS(nanoseconds=round(self.clock.t*1e9)))

    def spin(self, timeout_sec):
        dt = max(timeout_sec, 0.001)
        self.clock.t += dt
        speed = min(self.speed, 5*2*math.pi*0.085/60)
        if self.mode == 'stalled':
            speed = 0.0
        if self.mode == 'reverse':
            speed = -speed
        self.x += speed*dt
        for topic, callback in self.callbacks.items():
            msg = Odometry()
            stamp = self.clock.t - (2 if self.mode == 'stale' else 0)
            msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(round(stamp*1e9), 10**9)
            msg.header.frame_id, msg.child_frame_id = 'odom', 'base_link'
            msg.pose.pose.position.x = self.x
            msg.pose.pose.orientation.w = 1.0
            msg.twist.twist.linear.x = speed
            if self.mode == 'pose_jump' and self.speed and topic == '/odometry/filtered':
                msg.pose.pose.position.x += 1.0
            if self.mode == 'feedback_lost' and topic == '/odom':
                msg.twist.covariance[0] = 1e6
            callback(msg)


class GroundTest(unittest.TestCase):
    def test_40cm(self):
        output = self.run_case('normal', distance=0.40)
        self.assertIn('target=0.40m', output)

    def test_invalid_distance(self):
        with self.assertRaises(ValueError):
            ground.GroundCheck(None, distance=1.0)

    def run_case(self, mode, distance=0.20):
        clock = Clock()
        node = FakeNode(clock, mode)
        with patch.object(ground, 'time', clock), \
                patch.object(rclpy, 'spin_once', lambda n, timeout_sec: node.spin(timeout_sec)), \
                patch.object(rclpy, 'spin_until_future_complete', lambda *a, **kw: None), \
                patch.object(rclpy, 'ok', lambda: True), \
                contextlib.redirect_stdout(io.StringIO()) as output:
            result = ground.GroundCheck(node, distance=distance).run()
        self.assertEqual(node.sent[-1][1:], (0.0, 0.0))
        self.assertTrue(all(v in (0.0, 0.04) and w == 0.0 for _, v, w in node.sent))
        moving = [t for t, v, _ in node.sent if v]
        if mode == 'normal':
            self.assertEqual(result, 0)
            self.assertTrue(distance <= node.x < distance + 0.01)
            self.assertIn('GROUND_DIAGNOSTIC_COMPLETED=True', output.getvalue())
        else:
            self.assertEqual(result, 2)
        if mode in ('wrong_limit', 'other_publisher', 'stale', 'feedback_lost'):
            self.assertFalse(moving)
        if moving:
            first_stop = next(t for t, v, _ in node.sent if t > moving[0] and v == 0)
            self.assertLess(first_stop-moving[0], (10.1 if distance == 0.20 else 18.1))
        return output.getvalue()

    def test_normal_reaches_20cm_then_zero(self):
        self.run_case('normal')

    def test_wrong_limit_blocks_motion(self):
        self.run_case('wrong_limit')

    def test_other_publisher_blocks_motion(self):
        self.run_case('other_publisher')

    def test_stale_odometry_blocks_motion(self):
        self.run_case('stale')

    def test_invalid_actual_feedback_blocks_motion(self):
        self.run_case('feedback_lost')

    def test_ekf_jump_interrupts_motion(self):
        self.run_case('pose_jump')

    def test_reverse_progress_interrupts_motion(self):
        self.run_case('reverse')

    def test_motion_timeout_sends_zero(self):
        self.assertIn('not reached within motion timeout', self.run_case('stalled'))

    def test_stop_failure_not_reported_as_success(self):
        self.assertIn('GROUND_DIAGNOSTIC_COMPLETED=False', self.run_case('stop_failure'))


if __name__ == '__main__':
    unittest.main()
