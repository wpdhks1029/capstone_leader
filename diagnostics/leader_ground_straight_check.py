"""Bounded 20cm ground straight check with the diagnostic polling/EKF stack.

Default mode only prints the plan. Hardware execution requires --run-ground-test.
Commands are exclusively linear.x=0 or 0.04m/s and angular.z=0. No Nav2 action.
Requires clear space, a measured start mark, physical observation and power cutoff.
"""
import argparse
import math
import signal
import time

import rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy
from rclpy.signals import SignalHandlerOptions
from rcl_interfaces.srv import GetParameters
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


def stamp_seconds(msg):
    return msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9


def yaw(msg):
    q = msg.pose.pose.orientation
    return math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))


def delta(first, last):
    return (last.pose.pose.position.x-first.pose.pose.position.x,
            last.pose.pose.position.y-first.pose.pose.position.y)


class GroundCheck:
    def __init__(self, node, distance=0.20):
        if distance not in (0.20, 0.40):
            raise ValueError("Distance must be 0.20 or 0.40m")
        self.distance = distance
        self.motion_timeout = 10.0 if distance == 0.20 else 18.0
        self.node = node
        self.samples = {'raw': [], 'ekf': []}
        self.pub = node.create_publisher(
            Twist, '/cmd_vel', QoSProfile(depth=10, durability=DurabilityPolicy.VOLATILE))
        for key, topic in [('raw', '/odom'), ('ekf', '/odometry/filtered')]:
            node.create_subscription(Odometry, topic,
                                     lambda msg, k=key: self.samples[k].append(msg), 10)

    def parameters(self, name, names):
        client = self.node.create_client(GetParameters, f'/{name}/get_parameters')
        if not client.wait_for_service(timeout_sec=8):
            raise RuntimeError(f'{name} parameter service unavailable')
        request = GetParameters.Request()
        request.names = names
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=5)
        if not future.done() or future.result() is None:
            raise RuntimeError(f'{name} parameter request failed')
        values = future.result().values
        if len(values) != len(names):
            raise RuntimeError('Incomplete parameter response')
        return values

    def owners(self):
        other = [p.node_name for p in self.node.get_publishers_info_by_topic('/cmd_vel')
                 if p.node_name != self.node.get_name() or p.node_namespace != '/']
        if other:
            raise RuntimeError(f'Other command publishers: {other}')
        subscribers = self.node.get_subscriptions_info_by_topic('/cmd_vel')
        motors = [s for s in subscribers
                  if s.node_name == 'md200t_controller' and s.node_namespace == '/']
        unknown = [s.node_name for s in subscribers if s.node_namespace != '/'
                   or s.node_name not in ('md200t_controller', 'rosbag2_recorder')]
        if len(motors) != 1 or unknown:
            raise RuntimeError('Expected one MD200T command subscriber and optional recorder')

    def preflight(self):
        p = self.parameters('md200t_controller', [
            'allow_motion', 'max_rpm', 'dry_run', 'constrain_lateral_velocity',
            'feedback_poll_rate'])
        if not (p[0].bool_value and p[1].integer_value == 5 and not p[2].bool_value
                and p[3].bool_value and 5 <= p[4].double_value <= 10):
            raise RuntimeError('Requires motion enabled, max_rpm=5, real polling and lateral constraint')
        e = self.parameters('ekf_filter_node', ['odom0_config', 'world_frame', 'use_sim_time'])
        selection = e[0].bool_array_value
        if not (len(selection) == 15 and selection[6] and selection[7]
                and e[1].string_value == 'odom' and not e[2].bool_value):
            raise RuntimeError('Expected diagnostic vx+vy EKF in odom, using real time')
        self.owners()

    def guard(self, first=None):
        self.owners()
        now = self.node.get_clock().now().nanoseconds / 1e9
        for key, messages in self.samples.items():
            if not messages:
                raise RuntimeError(f'No {key} odometry')
            m = messages[-1]
            if m.header.frame_id != 'odom' or m.child_frame_id != 'base_link':
                raise RuntimeError(f'Unexpected {key} frames')
            q = m.pose.pose.orientation
            values = (m.pose.pose.position.x, m.pose.pose.position.y,
                      m.twist.twist.linear.x, m.twist.twist.linear.y,
                      m.twist.twist.angular.z, m.pose.covariance[0], m.pose.covariance[7],
                      q.x, q.y, q.z, q.w)
            if not all(math.isfinite(v) for v in values):
                raise RuntimeError(f'Non-finite {key} state')
            if not -0.1 <= now-stamp_seconds(m) <= 0.4:
                raise RuntimeError(f'Stale {key} odometry')
            if abs(m.twist.twist.linear.x) > 0.08 or abs(m.twist.twist.angular.z) > 0.2:
                raise RuntimeError(f'Unexpected {key} speed')
            if key == 'raw' and m.twist.covariance[0] >= 1000:
                raise RuntimeError('Actual wheel feedback invalid')
            if key == 'ekf' and max(m.pose.covariance[0], m.pose.covariance[7]) > 100:
                raise RuntimeError('EKF uncertainty too large for diagnostic test')
            if len(messages) > 1:
                previous = messages[-2]
                dx, dy = delta(previous, m)
                dt = max(0.0, stamp_seconds(m)-stamp_seconds(previous))
                if math.hypot(dx, dy) > 0.02 + 0.08*dt:
                    raise RuntimeError(f'{key} position jump')
            if first is not None:
                dx, dy = delta(first[key], m)
                angle = yaw(m)-yaw(first[key])
                angle = math.atan2(math.sin(angle), math.cos(angle))
                if abs(angle) > 0.15:
                    raise RuntimeError(f'{key} heading changed more than 0.15rad')
                lateral = -dx*math.sin(yaw(first[key])) + dy*math.cos(yaw(first[key]))
                forward = dx*math.cos(yaw(first[key])) + dy*math.sin(yaw(first[key]))
                if abs(lateral) > 0.05 or forward < -0.01 or math.hypot(dx, dy) > self.distance + 0.05:
                    raise RuntimeError(f'{key} outside straight-test displacement bounds')

    def publish(self, speed):
        msg = Twist()
        msg.linear.x = speed
        self.pub.publish(msg)

    def warmup(self):
        end, next_pub = time.monotonic()+2, time.monotonic()
        while time.monotonic() < end:
            self.owners()
            if time.monotonic() >= next_pub:
                self.publish(0.0)
                next_pub = time.monotonic()+0.1
            rclpy.spin_once(self.node, timeout_sec=0.01)

    def run(self):
        first, reached, failed = None, False, False
        try:
            self.preflight()
            print('PRECHECK PASSED; zero-speed warmup', flush=True)
            self.warmup()
            self.guard()
            first = {key: rows[-1] for key, rows in self.samples.items()}
            print(f'FORWARD target={self.distance:.2f}m, command=0.04m/s, max duration={self.motion_timeout:g}s', flush=True)
            end, next_pub = time.monotonic()+self.motion_timeout, time.monotonic()
            while time.monotonic() < end:
                self.guard(first)
                dx, dy = delta(first['raw'], self.samples['raw'][-1])
                forward = dx*math.cos(yaw(first['raw'])) + dy*math.sin(yaw(first['raw']))
                if forward >= self.distance:
                    reached = True
                    break
                if time.monotonic() >= next_pub:
                    self.publish(0.04)
                    next_pub = time.monotonic()+0.1
                rclpy.spin_once(self.node, timeout_sec=0.01)
            if not reached:
                raise RuntimeError('Distance target not reached within motion timeout')
        except (RuntimeError, KeyboardInterrupt) as exc:
            failed = True
            print(f'TEST INTERRUPTED/FAILED: {exc}', flush=True)
        finally:
            print('ZERO-SPEED shutdown phase', flush=True)
            end = time.monotonic()+2
            while rclpy.ok() and time.monotonic() < end:
                self.publish(0.0)
                rclpy.spin_once(self.node, timeout_sec=0.05)
            stopped = False
            try:
                self.guard(first)
                last = self.samples['raw'][-1]
                stopped = (abs(last.twist.twist.linear.x) < 0.016
                           and abs(last.twist.twist.angular.z) < 0.04)
            except RuntimeError as exc:
                failed = True
                print(f'FINAL STATE INVALID: {exc}', flush=True)
            print(f'FINAL_RAW_ODOM_NEAR_ZERO={stopped}', flush=True)
            error = float('inf')
            if first is not None and all(self.samples.values()):
                shifts = {key: delta(first[key], rows[-1]) for key, rows in self.samples.items()}
                for key, (dx, dy) in shifts.items():
                    print(f'{key}: delta_xy=({dx:.4f},{dy:.4f})m distance={math.hypot(dx,dy):.4f}m',
                          flush=True)
                error = math.hypot(shifts['raw'][0]-shifts['ekf'][0],
                                   shifts['raw'][1]-shifts['ekf'][1])
                print(f'RAW_EKF_DELTA_DIFFERENCE={error:.4f}m', flush=True)
            complete = reached and stopped and not failed and error <= 0.03
            print(f'GROUND_DIAGNOSTIC_COMPLETED={complete}', flush=True)
            print('Measure actual travel with the ruler; odometry is not ground truth.', flush=True)
        return 0 if complete else 2


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run-ground-test', action='store_true')
    parser.add_argument('--distance', type=float, choices=(0.20, 0.40), default=0.20)
    args = parser.parse_args()
    if not args.run_ground_test:
        print(f'PLAN ONLY: {args.distance:.2f}m target, +0.04m/s only, '
              f'{10 if args.distance == 0.20 else 18}s motion timeout, then zero. No ROS init.')
        return 0
    def interrupt(signum, frame):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, interrupt)
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = rclpy.create_node('leader_ground_straight_check')
    try:
        return GroundCheck(node, distance=args.distance).run()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
