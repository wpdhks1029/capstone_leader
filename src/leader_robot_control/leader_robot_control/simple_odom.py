#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped

from tf2_ros import TransformBroadcaster


class SimpleOdom(Node):

    def __init__(self):
        super().__init__('simple_odom')

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        self.linear = 0.0
        self.angular = 0.0

        self.last_time = self.get_clock().now()

        self.cmd_timeout = 0.5
        self.last_cmd_time = self.get_clock().now()

        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_callback,
            10
        )

        self.odom_pub = self.create_publisher(
            Odometry,
            '/odom',
            10
        )

        self.tf_broadcaster = TransformBroadcaster(self)

        self.timer = self.create_timer(
            0.02,
            self.update
        )

        self.get_logger().info('Simple odometry started')
        self.get_logger().info('Publishing: /odom')
        self.get_logger().info('TF: odom -> base_link')


    def cmd_callback(self, msg):

        self.linear = float(msg.linear.x)
        self.angular = float(msg.angular.z)

        self.last_cmd_time = self.get_clock().now()


    def update(self):

        now = self.get_clock().now()

        dt = (now - self.last_time).nanoseconds / 1e9

        self.last_time = now

        if dt <= 0.0 or dt > 1.0:
            return

        cmd_age = (
            now - self.last_cmd_time
        ).nanoseconds / 1e9

        if cmd_age > self.cmd_timeout:
            linear = 0.0
            angular = 0.0
        else:
            linear = self.linear
            angular = self.angular


        # ============================================
        # Differential-drive odometry integration
        # ============================================

        self.x += (
            linear *
            math.cos(self.yaw) *
            dt
        )

        self.y += (
            linear *
            math.sin(self.yaw) *
            dt
        )

        self.yaw += angular * dt

        # yaw normalization
        self.yaw = math.atan2(
            math.sin(self.yaw),
            math.cos(self.yaw)
        )


        # ============================================
        # Quaternion
        # ============================================

        qz = math.sin(self.yaw / 2.0)
        qw = math.cos(self.yaw / 2.0)


        # ============================================
        # odom -> base_link TF
        # ============================================

        transform = TransformStamped()

        transform.header.stamp = now.to_msg()
        transform.header.frame_id = 'odom'
        transform.child_frame_id = 'base_link'

        transform.transform.translation.x = self.x
        transform.transform.translation.y = self.y
        transform.transform.translation.z = 0.0

        transform.transform.rotation.x = 0.0
        transform.transform.rotation.y = 0.0
        transform.transform.rotation.z = qz
        transform.transform.rotation.w = qw

        self.tf_broadcaster.sendTransform(transform)


        # ============================================
        # /odom
        # ============================================

        odom = Odometry()

        odom.header.stamp = now.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0

        odom.pose.pose.orientation.x = 0.0
        odom.pose.pose.orientation.y = 0.0
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw

        odom.twist.twist.linear.x = linear
        odom.twist.twist.angular.z = angular

        self.odom_pub.publish(odom)


def main(args=None):

    rclpy.init(args=args)

    node = SimpleOdom()

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
