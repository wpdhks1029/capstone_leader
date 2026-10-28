"""Offline bridge regression checks; no ROS context, nodes, or devices."""
import copy
import importlib.util
from pathlib import Path
from types import SimpleNamespace
import unittest

from builtin_interfaces.msg import Time
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry, Path as PathMessage


SOURCE = Path(__file__).resolve().parents[1] / 'convoy_robot/real_leader_bridge_node.py'
SPEC = importlib.util.spec_from_file_location('leader_bridge_under_test', SOURCE)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)
Bridge = MODULE.RealLeaderBridge


class Publisher:
    def __init__(self):
        self.messages = []

    def publish(self, msg):
        self.messages.append(copy.deepcopy(msg))


class BridgeFreshnessTest(unittest.TestCase):
    def setUp(self):
        self.now = SimpleNamespace(nanoseconds=100_000_000_000)
        self.tf = TransformStamped()
        self.tf.header.stamp = Time(sec=99, nanosec=900_000_000)
        self.tf.transform.rotation.w = 1.0
        odom = Odometry()
        odom.header.stamp = Time(sec=99, nanosec=950_000_000)
        odom.twist.twist.linear.x = 0.12
        self.node = SimpleNamespace(
            map_frame='map', base_frame='base_link',
            tf_timeout=0.5, odom_timeout=0.5,
            path_min_distance=0.1, max_path_points=3,
            latest_filtered_odom=odom, path=PathMessage(),
            last_path_x=None, last_path_y=None,
            leader_odom_pub=Publisher(), leader_path_pub=Publisher(),
            stamp_is_fresh=Bridge.stamp_is_fresh,
            get_clock=lambda: SimpleNamespace(now=lambda: self.now),
            get_logger=lambda: SimpleNamespace(warning=lambda *a, **kw: None),
            tf_buffer=SimpleNamespace(lookup_transform=lambda *a, **kw: self.tf),
        )
        self.node.path.header.frame_id = 'map'

    def publish(self):
        Bridge.publish_leader_state(self.node)

    def assert_suppressed(self):
        self.publish()
        self.assertEqual(len(self.node.leader_odom_pub.messages), 0)
        self.assertEqual(len(self.node.leader_path_pub.messages), 0)
        self.assertEqual(len(self.node.path.poses), 0)

    def test_fresh_input_preserves_measurement_stamp_and_twist(self):
        self.publish()
        out = self.node.leader_odom_pub.messages[-1]
        self.assertEqual(out.header.stamp, self.tf.header.stamp)
        self.assertEqual(out.header.frame_id, 'map')
        self.assertEqual(out.child_frame_id, 'base_link')
        self.assertAlmostEqual(out.twist.twist.linear.x, 0.12)
        self.assertEqual(self.node.path.header.stamp, self.tf.header.stamp)

    def test_no_odometry_suppresses_publication(self):
        self.node.latest_filtered_odom = None
        self.assert_suppressed()

    def test_stale_tf_suppresses_publication(self):
        self.tf.header.stamp = Time(sec=99)
        self.assert_suppressed()

    def test_stale_odometry_suppresses_publication(self):
        self.node.latest_filtered_odom.header.stamp = Time(sec=99)
        self.assert_suppressed()

    def test_missing_tf_suppresses_publication(self):
        def missing(*args, **kwargs):
            raise MODULE.TransformException('missing transform')
        self.node.tf_buffer.lookup_transform = missing
        self.assert_suppressed()

    def test_zero_and_future_stamps_are_rejected(self):
        for stamp in [Time(), Time(sec=101)]:
            with self.subTest(stamp=stamp):
                self.tf.header.stamp = stamp
                self.assert_suppressed()

    def test_timeout_boundary(self):
        self.assertTrue(Bridge.stamp_is_fresh(
            Time(sec=99, nanosec=500_000_000), self.now, 0.5))
        self.assertFalse(Bridge.stamp_is_fresh(
            Time(sec=99, nanosec=499_999_999), self.now, 0.5))

    def test_loss_then_recovery(self):
        self.publish()
        self.now.nanoseconds = 102_000_000_000
        self.publish()
        self.assertEqual(len(self.node.leader_odom_pub.messages), 1)
        self.assertEqual(len(self.node.path.poses), 1)
        self.tf.header.stamp = Time(sec=102)
        self.node.latest_filtered_odom.header.stamp = Time(sec=102)
        self.publish()
        self.assertEqual(len(self.node.leader_odom_pub.messages), 2)
        self.assertEqual(len(self.node.path.poses), 1)

    def test_sampling_republication_and_trimming(self):
        for x in [0.0, 0.05, 0.11, 0.22, 0.33, 0.44]:
            self.tf.transform.translation.x = x
            self.publish()
        self.assertEqual(len(self.node.leader_path_pub.messages), 6)
        self.assertEqual(len(self.node.path.poses), 3)
        self.assertAlmostEqual(self.node.path.poses[0].pose.position.x, 0.22)


if __name__ == '__main__':
    unittest.main()
