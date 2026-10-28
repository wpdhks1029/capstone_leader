"""Fake-serial tests of diagnostic polling and inherited odometry gates."""
import importlib.util
import math
from pathlib import Path
import struct
from types import MethodType, SimpleNamespace as NS
import unittest
from unittest.mock import patch

from rclpy.time import Time
import serial
import leader_robot_control.md200t_controller as baseline


SOURCE = Path(__file__).resolve().parents[1] / 'md200t_polling_controller.py'
SPEC = importlib.util.spec_from_file_location('polling_under_test', SOURCE)
polling = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(polling)


class FeedbackScaleTest(unittest.TestCase):
    def test_scale_only_during_odometry_and_restore(self):
        node = object.__new__(polling.PollingMD200TController)
        node.actual_m1_rpm, node.actual_m2_rpm = 4, 5
        node.feedback_speed_scale = 1.12
        seen = []
        with patch.object(baseline.MD200TController, 'odom_callback',
                          lambda n: seen.append((n.actual_m1_rpm, n.actual_m2_rpm))):
            node.odom_callback()
        self.assertAlmostEqual(seen[0][0], 4.48)
        self.assertAlmostEqual(seen[0][1], 5.6)
        self.assertEqual((node.actual_m1_rpm, node.actual_m2_rpm), (4, 5))
        with patch.object(baseline.MD200TController, 'odom_callback', side_effect=RuntimeError):
            with self.assertRaises(RuntimeError):
                node.odom_callback()
        self.assertEqual((node.actual_m1_rpm, node.actual_m2_rpm), (4, 5))


class Clock:
    def __init__(self):
        self.t = 100.0

    def time(self):
        return self.t

    def sleep(self, seconds):
        self.t += seconds

    def now(self):
        return Time(nanoseconds=round(self.t * 1e9))


def response(left, right):
    data = bytearray(14)
    struct.pack_into('<h', data, 0, left)
    struct.pack_into('<h', data, 7, right)
    body = bytes.fromhex('b8 b7 01 d8 0e') + data
    return body + bytes([(-sum(body)) & 255])


class FakeSerial:
    def __init__(self, clock):
        self.clock = clock
        self.mode = 'normal'
        self.rpm = (5, 5)
        self.sent, self.chunks = [], []
        self.buffer = bytearray()

    def reset_input_buffer(self):
        self.buffer.clear()
        self.chunks.clear()

    def write(self, packet):
        self.sent.append(packet)
        if self.mode == 'error':
            raise serial.SerialException('fake IO failure')
        if self.mode == 'partial_write':
            return 3
        frame = response(*self.rpm)
        if self.mode == 'checksum':
            frame = frame[:-1] + bytes([frame[-1] ^ 1])
        if self.mode == 'noise':
            frame = b'x' + frame
        if self.mode != 'missing':
            self.chunks = [(self.clock.t + 0.005, frame[:7]),
                           (self.clock.t + 0.015, frame[7:])]
        return len(packet)

    def flush(self):
        pass

    @property
    def in_waiting(self):
        while self.chunks and self.chunks[0][0] <= self.clock.t:
            self.buffer.extend(self.chunks.pop(0)[1])
        return len(self.buffer)

    def read(self, count):
        result = bytes(self.buffer[:count])
        del self.buffer[:count]
        return result


class PollingTest(unittest.TestCase):
    def setUp(self):
        self.clock = Clock()
        self.serial = FakeSerial(self.clock)
        self.messages = []
        logger = NS(info=lambda *a, **k: None, warning=lambda *a, **k: None,
                    error=lambda *a, **k: None)
        self.node = NS(
            dry_run=False, allow_motion=False, ser=self.serial, controller_id=1,
            poll_requests=0, poll_valid=0, poll_missing=0, poll_errors=0,
            actual_m1_rpm=0, actual_m2_rpm=0, last_feedback_time=None,
            feedback_timeout=0.3, wheel_radius=0.085, wheel_track=0.466,
            invert_left=False, invert_right=False, odom_x=0.0, odom_y=0.0,
            odom_yaw=0.0, last_odom_time=self.clock.now(),
            odom_frame_id='odom', base_frame_id='base_link',
            odom_pub=NS(publish=self.messages.append),
            get_logger=lambda: logger, get_clock=lambda: self.clock,
        )
        for name in ('read_feedback_frame', 'parse_feedback'):
            setattr(self.node, name, MethodType(getattr(baseline.MD200TController, name), self.node))
        self.patch = patch.object(baseline, 'time', self.clock)
        self.patch.start()
        self.addCleanup(self.patch.stop)

    def poll(self):
        polling.PollingMD200TController.poll_feedback(self.node)

    def test_pid4_bytes_and_partial_frame(self):
        self.poll()
        self.assertEqual(self.serial.sent, [bytes.fromhex('b7 b8 01 04 01 d8 b3')])
        self.assertEqual(self.node.poll_valid, 1)
        self.assertEqual((self.node.actual_m1_rpm, self.node.actual_m2_rpm), (5, 5))
        self.assertIsNotNone(self.node.last_feedback_time)

    def test_noise_before_frame(self):
        self.serial.mode = 'noise'
        self.poll()
        self.assertEqual(self.node.poll_valid, 1)

    def test_checksum_failure_does_not_refresh_feedback(self):
        self.serial.mode = 'checksum'
        self.poll()
        self.assertEqual(self.node.poll_missing, 1)
        self.assertIsNone(self.node.last_feedback_time)

    def test_missing_then_recovery_preserves_stale_gate(self):
        self.poll()
        good_stamp = self.node.last_feedback_time
        self.clock.sleep(0.4)
        self.serial.mode = 'missing'
        self.poll()
        self.assertIs(self.node.last_feedback_time, good_stamp)
        self.assertEqual(self.node.actual_m1_rpm, 5)  # not fabricated zero feedback
        baseline.MD200TController.odom_callback(self.node)
        self.assertEqual(self.messages[-1].twist.covariance[0], 1e6)
        self.assertEqual(self.messages[-1].twist.twist.linear.x, 0)
        self.serial.mode = 'normal'
        self.clock.sleep(0.1)
        self.poll()
        baseline.MD200TController.odom_callback(self.node)
        self.assertEqual(self.messages[-1].twist.covariance[0], 0)
        self.assertAlmostEqual(self.messages[-1].twist.twist.linear.x, 5*2*math.pi*0.085/60)

    def test_dry_run_sends_nothing(self):
        self.node.dry_run = True
        self.poll()
        self.assertFalse(self.serial.sent)

    def test_partial_write_and_io_failure(self):
        for mode in ('partial_write', 'error'):
            with self.subTest(mode=mode):
                self.serial.mode = mode
                previous = self.node.poll_errors
                self.poll()
                self.assertEqual(self.node.poll_errors, previous + 1)
                self.assertIsNone(self.node.last_feedback_time)

    def test_same_serial_owner_and_motion_gate_remain(self):
        self.poll()
        baseline.MD200TController.send_packet(self.node, 'fake command', 5, 5)
        self.poll()
        self.assertIs(self.node.ser, self.serial)
        self.assertEqual([p[3] for p in self.serial.sent], [4, 207, 4])
        self.assertEqual(self.serial.sent[1], baseline.make_velocity_packet(1, 0, 0))

    def test_idle_polling_keeps_actual_feedback_fresh(self):
        self.serial.rpm = (0, 0)
        for _ in range(100):
            self.poll()
            self.clock.sleep(0.085)
            baseline.MD200TController.odom_callback(self.node)
        self.assertEqual(self.node.poll_valid, 100)
        self.assertTrue(all(m.twist.covariance[0] == 0 for m in self.messages))
        self.assertTrue(all(p[3] == 4 for p in self.serial.sent))

    def test_lateral_constraint_preserves_real_forward_velocity_and_stale_gate(self):
        original_publisher = self.node.odom_pub
        self.node.odom_pub = polling.LateralConstraintPublisher(original_publisher, 0.0004)
        self.poll()
        baseline.MD200TController.odom_callback(self.node)
        self.assertEqual(self.messages[-1].twist.covariance[7], 0.0004)
        self.assertAlmostEqual(self.messages[-1].twist.twist.linear.x, 5*2*math.pi*0.085/60)
        self.clock.sleep(0.4)
        baseline.MD200TController.odom_callback(self.node)
        self.assertEqual(self.messages[-1].twist.covariance[0], 1e6)
        self.assertEqual(self.messages[-1].twist.covariance[7], 0.0004)
        self.assertIs(self.node.odom_pub.publisher, original_publisher)

    def test_constraint_rejects_invalid_variance_or_nonzero_lateral_input(self):
        for variance in (0, -1, float('nan'), float('inf')):
            with self.assertRaises(ValueError):
                polling.LateralConstraintPublisher(self.node.odom_pub, variance)
        from nav_msgs.msg import Odometry
        msg = Odometry()
        msg.twist.twist.linear.y = 0.1
        wrapper = polling.LateralConstraintPublisher(self.node.odom_pub, 0.0004)
        with self.assertRaises(ValueError):
            wrapper.publish(msg)
        self.assertFalse(self.messages)


if __name__ == '__main__':
    unittest.main()
