"""One straight lifted-wheel ROS pulse; default mode does not initialize ROS.

Requires the dedicated leader_lifted_ros_check launch, max_rpm=5, wheels lifted.
Publishes zero for 2 s, forward 0.04 m/s for about 1.5 s, then zero for 2 s.
Reports encoder-derived motion, not physical robot travel. No Nav2 actions.
"""
import argparse
import math
import signal
import time


def displacement(first, last):
    return math.hypot(last.pose.pose.position.x - first.pose.pose.position.x,
                      last.pose.pose.position.y - first.pose.pose.position.y)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run-lifted-test', action='store_true')
    args = parser.parse_args()
    if not args.run_lifted_test:
        print('OFFLINE ONLY: zero 2s -> forward 0.04m/s 1.5s -> zero 2s; no ROS/device access')
        return 0

    import rclpy
    from rclpy.signals import SignalHandlerOptions
    from rclpy.qos import QoSProfile, DurabilityPolicy
    from rcl_interfaces.srv import GetParameters
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry

    def interrupt(signum, frame):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, interrupt)
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = rclpy.create_node('leader_lifted_pulse_check')
    qos = QoSProfile(depth=10, durability=DurabilityPolicy.VOLATILE)
    pub = node.create_publisher(Twist, '/cmd_vel', qos)
    samples = {'raw': [], 'ekf': []}
    for key, topic in [('raw', '/odom'), ('ekf', '/odometry/filtered')]:
        node.create_subscription(Odometry, topic,
                                 lambda msg, k=key: samples[k].append(msg), 10)

    def no_other_command_publishers():
        others = [p.node_name for p in node.get_publishers_info_by_topic('/cmd_vel')
                  if p.node_name != node.get_name() or p.node_namespace != '/']
        if others:
            raise RuntimeError(f'Other command publishers detected: {others}')

    def phase(speed, seconds, validate=False):
        end = time.monotonic() + seconds
        next_pub = time.monotonic()
        while time.monotonic() < end:
            no_other_command_publishers()
            if validate:
                now = node.get_clock().now().nanoseconds
                for key in samples:
                    if not samples[key]:
                        raise RuntimeError(f'No {key} odometry')
                    msg = samples[key][-1]
                    values = (msg.pose.pose.position.x, msg.pose.pose.position.y,
                              msg.twist.twist.linear.x, msg.twist.twist.linear.y,
                              msg.twist.twist.angular.z,
                              msg.pose.covariance[0], msg.pose.covariance[7])
                    if any(not math.isfinite(value) for value in values):
                        raise RuntimeError(f'Non-finite {key} state or uncertainty')
                    # Broad diagnostic ceiling, not a calibrated accuracy limit.
                    if key == 'ekf' and max(msg.pose.covariance[0], msg.pose.covariance[7]) > 100:
                        raise RuntimeError('EKF uncertainty too large for this lifted-wheel test')
                    age = (now - msg.header.stamp.sec * 10**9
                           - msg.header.stamp.nanosec) / 1e9
                    if not -0.1 <= age <= 0.5:
                        raise RuntimeError(f'Stale {key} odometry: {age:.3f}s')
                    if key == 'raw' and msg.twist.covariance[0] >= 1000:
                        raise RuntimeError('Raw odometry reports invalid wheel feedback')
                    if (abs(msg.twist.twist.linear.x) > 0.08
                            or abs(msg.twist.twist.angular.z) > 0.2):
                        raise RuntimeError(f'Unexpected {key} speed during straight pulse')
                    if len(samples[key]) >= 2:
                        previous = samples[key][-2]
                        dt = ((msg.header.stamp.sec - previous.header.stamp.sec)
                              + (msg.header.stamp.nanosec - previous.header.stamp.nanosec) / 1e9)
                        step = displacement(previous, msg)
                        if step > 0.02 + 0.08 * max(0.0, dt):
                            raise RuntimeError(f'{key} position jump: {step:.3f}m in {dt:.3f}s')
            if time.monotonic() >= next_pub:
                msg = Twist()
                msg.linear.x = speed
                pub.publish(msg)
                next_pub = time.monotonic() + 0.1
            rclpy.spin_once(node, timeout_sec=0.01)

    moved = False
    try:
        # Read the live motion gate and RPM limit before sending any pulse.
        client = node.create_client(GetParameters, '/md200t_controller/get_parameters')
        if not client.wait_for_service(timeout_sec=10.0):
            raise RuntimeError('MD200T parameter service unavailable')
        request = GetParameters.Request()
        request.names = ['allow_motion', 'max_rpm', 'dry_run']
        future = client.call_async(request)
        rclpy.spin_until_future_complete(node, future, timeout_sec=5.0)
        if not future.done() or future.result() is None:
            raise RuntimeError('Cannot read MD200T parameters')
        values = future.result().values
        if not (values[0].bool_value and values[1].integer_value == 5
                and not values[2].bool_value):
            raise RuntimeError('Expected allow_motion=true, max_rpm=5, dry_run=false')
        subscribers = node.get_subscriptions_info_by_topic('/cmd_vel')
        motor_subscribers = [s for s in subscribers
                             if s.node_name == 'md200t_controller'
                             and s.node_namespace == '/']
        unexpected = [s.node_name for s in subscribers
                      if s.node_namespace != '/'
                      or s.node_name not in ('md200t_controller', 'rosbag2_recorder')]
        if len(motor_subscribers) != 1 or unexpected:
            raise RuntimeError('Expected one MD200T subscriber; only rosbag observers allowed')
        print('PRECHECK PASSED; zero-speed warmup', flush=True)
        phase(0.0, 2.0)
        if any(not samples[key] for key in samples):
            raise RuntimeError('Both raw and EKF odometry are required')
        first = {key: samples[key][-1] for key in samples}
        motion_index = {key: len(samples[key]) for key in samples}
        print('FORWARD 0.04m/s for about 1.5 seconds', flush=True)
        phase(0.04, 1.5, validate=True)
        for key in samples:
            if not any(m.twist.twist.linear.x > 0.005
                       for m in samples[key][motion_index[key]:]):
                raise RuntimeError(f'No forward motion observed in {key} odometry')
        moved = True
    except (RuntimeError, KeyboardInterrupt) as exc:
        print(f'TEST INTERRUPTED/FAILED: {exc}', flush=True)
    finally:
        # Best effort only; hardware watchdog and physical cutoff remain needed.
        print('ZERO-SPEED shutdown phase', flush=True)
        end = time.monotonic() + 2.0
        while rclpy.ok() and time.monotonic() < end:
            pub.publish(Twist())
            rclpy.spin_once(node, timeout_sec=0.05)
        stopped = False
        if samples['raw']:
            last = samples['raw'][-1]
            stamp = last.header.stamp
            age = (node.get_clock().now().nanoseconds
                   - stamp.sec * 10**9 - stamp.nanosec) / 1e9
            stopped = (0 <= age <= 0.5
                       and last.twist.covariance[0] < 1000
                       and abs(last.twist.twist.linear.x) < 0.016
                       and abs(last.twist.twist.angular.z) < 0.04)
        print(f'FINAL_RAW_ODOM_NEAR_ZERO={stopped}', flush=True)
        if moved:
            for key in samples:
                messages = samples[key][motion_index[key]:]
                max_v = max((abs(m.twist.twist.linear.x) for m in messages), default=0)
                print(f'{key}: messages={len(messages)}, max_abs_vx={max_v:.4f}m/s, '
                      f'wheel_derived_displacement={displacement(first[key], samples[key][-1]):.4f}m',
                      flush=True)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0 if moved and stopped else 2


if __name__ == '__main__':
    raise SystemExit(main())
