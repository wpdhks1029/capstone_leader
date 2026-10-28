#!/usr/bin/env python3
"""
Real leader bridge for the hierarchical convoy robot.

The proven real-hardware stack keeps its native topics/TF:
  /odometry/filtered + map->odom->base_link

This node adds a convoy-facing interface without changing the motor, EKF,
LiDAR, IMU, AMCL, or Nav2 stack:
  /leader/odom  : leader pose in the shared map frame + filtered twist
  /leader/path  : sampled leader path in the shared map frame
"""
import math

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from tf2_ros import Buffer, TransformException, TransformListener


class RealLeaderBridge(Node):
    def __init__(self):
        super().__init__('real_leader_bridge')

        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('filtered_odom_topic', '/odometry/filtered')
        self.declare_parameter('leader_odom_topic', '/leader/odom')
        self.declare_parameter('leader_path_topic', '/leader/path')
        self.declare_parameter('path_min_distance', 0.10)
        self.declare_parameter('publish_rate', 10.0)
        self.declare_parameter('max_path_points', 5000)
        self.declare_parameter('tf_timeout', 0.5)
        self.declare_parameter('odom_timeout', 0.5)

        self.map_frame = str(self.get_parameter('map_frame').value)
        self.base_frame = str(self.get_parameter('base_frame').value)
        self.filtered_odom_topic = str(
            self.get_parameter('filtered_odom_topic').value
        )
        self.leader_odom_topic = str(
            self.get_parameter('leader_odom_topic').value
        )
        self.leader_path_topic = str(
            self.get_parameter('leader_path_topic').value
        )
        self.path_min_distance = float(
            self.get_parameter('path_min_distance').value
        )
        self.publish_rate = float(
            self.get_parameter('publish_rate').value
        )
        self.max_path_points = int(
            self.get_parameter('max_path_points').value
        )
        self.tf_timeout = float(self.get_parameter('tf_timeout').value)
        self.odom_timeout = float(self.get_parameter('odom_timeout').value)
        if any(not math.isfinite(value) or value <= 0.0
               for value in (self.tf_timeout, self.odom_timeout)):
            raise ValueError('tf_timeout and odom_timeout must be finite and positive')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.latest_filtered_odom = None

        self.path = Path()
        self.path.header.frame_id = self.map_frame
        self.last_path_x = None
        self.last_path_y = None

        self.create_subscription(
            Odometry,
            self.filtered_odom_topic,
            self.filtered_odom_callback,
            20,
        )

        self.leader_odom_pub = self.create_publisher(
            Odometry,
            self.leader_odom_topic,
            20,
        )
        self.leader_path_pub = self.create_publisher(
            Path,
            self.leader_path_topic,
            10,
        )

        self.create_timer(
            1.0 / max(self.publish_rate, 1.0),
            self.publish_leader_state,
        )

        self.get_logger().info(
            'Real leader bridge started: '
            f'{self.map_frame}->{self.base_frame}, '
            f'{self.filtered_odom_topic} -> '
            f'{self.leader_odom_topic}, {self.leader_path_topic}'
        )

    def filtered_odom_callback(self, msg):
        self.latest_filtered_odom = msg

    @staticmethod
    def stamp_is_fresh(stamp, now, timeout):
        stamp_ns = stamp.sec * 1000000000 + stamp.nanosec
        age = (now.nanoseconds - stamp_ns) / 1e9
        # Reject missing stamps and clock jumps; allow 100 ms clock skew.
        return stamp_ns > 0 and -0.1 <= age <= timeout

    def publish_leader_state(self):
        if self.latest_filtered_odom is None:
            return

        try:
            transform = self.tf_buffer.lookup_transform(
                self.map_frame,
                self.base_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=0.05),
            )
        except TransformException as exc:
            self.get_logger().warning(
                f'Waiting for TF {self.map_frame}->{self.base_frame}: {exc}',
                throttle_duration_sec=2.0,
            )
            return

        # Check after the potentially blocking TF lookup. A cached TF or twist
        # must not be presented to the follower as a new measurement.
        now = self.get_clock().now()
        if not self.stamp_is_fresh(transform.header.stamp, now, self.tf_timeout):
            self.get_logger().warning(
                'Leader publication paused: stale or invalid TF stamp',
                throttle_duration_sec=2.0,
            )
            return
        if not self.stamp_is_fresh(
                self.latest_filtered_odom.header.stamp, now, self.odom_timeout):
            self.get_logger().warning(
                'Leader publication paused: stale or invalid filtered odometry stamp',
                throttle_duration_sec=2.0,
            )
            return

        leader_odom = Odometry()
        leader_odom.header.stamp = transform.header.stamp
        leader_odom.header.frame_id = self.map_frame
        leader_odom.child_frame_id = self.base_frame

        leader_odom.pose.pose.position.x = transform.transform.translation.x
        leader_odom.pose.pose.position.y = transform.transform.translation.y
        leader_odom.pose.pose.position.z = transform.transform.translation.z
        leader_odom.pose.pose.orientation = transform.transform.rotation

        if self.latest_filtered_odom is not None:
            leader_odom.twist = self.latest_filtered_odom.twist

        self.leader_odom_pub.publish(leader_odom)

        x = leader_odom.pose.pose.position.x
        y = leader_odom.pose.pose.position.y
        moved_enough = (
            self.last_path_x is None
            or math.hypot(x - self.last_path_x, y - self.last_path_y)
            >= self.path_min_distance
        )

        if moved_enough:
            pose = PoseStamped()
            pose.header = leader_odom.header
            pose.pose = leader_odom.pose.pose
            self.path.poses.append(pose)

            if len(self.path.poses) > self.max_path_points:
                excess = len(self.path.poses) - self.max_path_points
                del self.path.poses[:excess]

            self.last_path_x = x
            self.last_path_y = y

        # Republish periodically so a follower that joins later receives the
        # current path even while the leader is stopped.
        self.path.header.stamp = transform.header.stamp
        self.leader_path_pub.publish(self.path)


def main(args=None):
    rclpy.init(args=args)
    node = RealLeaderBridge()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
