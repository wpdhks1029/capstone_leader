"""Read-only odometry observer. Publishes no velocity commands."""
import argparse
import math
import time

import rclpy
from nav_msgs.msg import Odometry


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--seconds', type=float, default=30.0)
    args = parser.parse_args()
    if not math.isfinite(args.seconds) or not 5 <= args.seconds <= 300:
        parser.error('--seconds must be finite and between 5 and 300')
    rclpy.init()
    node = rclpy.create_node('leader_idle_ekf_watch')
    streams = {'raw': [], 'ekf': []}
    for key, topic in [('raw', '/odom'), ('ekf', '/odometry/filtered')]:
        node.create_subscription(Odometry, topic,
                                 lambda msg, k=key: streams[k].append(msg), 10)
    print(f'Observing {args.seconds:g} seconds; no commands will be published...', flush=True)
    start = time.monotonic()
    end = start + args.seconds
    next_progress = start + 30
    passed = True
    try:
        while rclpy.ok() and time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.1)
            if time.monotonic() >= next_progress and time.monotonic() < end:
                print(f'Elapsed {time.monotonic()-start:.0f}s; '
                      f'raw={len(streams["raw"])} ekf={len(streams["ekf"])}', flush=True)
                next_progress += 30
        now = node.get_clock().now().nanoseconds
        for key, messages in streams.items():
            if not messages:
                print(key, 'NO_MESSAGES')
                passed = False
                continue
            first, last = messages[0], messages[-1]
            a, b = first.pose.pose.position, last.pose.pose.position
            delta = math.hypot(b.x-a.x, b.y-a.y)
            stamp_ns = last.header.stamp.sec * 10**9 + last.header.stamp.nanosec
            age = (now-stamp_ns)/1e9
            steps = [math.hypot(b.pose.pose.position.x-a.pose.pose.position.x,
                                b.pose.pose.position.y-a.pose.pose.position.y)
                     for a, b in zip(messages, messages[1:])]
            gaps = [(b.header.stamp.sec-a.header.stamp.sec)
                    + (b.header.stamp.nanosec-a.header.stamp.nanosec)/1e9
                    for a, b in zip(messages, messages[1:])]
            max_step, max_gap = max(steps, default=0), max(gaps, default=0)
            max_vx = max(abs(m.twist.twist.linear.x) for m in messages)
            finite = all(math.isfinite(value) for m in messages for value in (
                m.pose.pose.position.x, m.pose.pose.position.y,
                m.twist.twist.linear.x, m.twist.twist.angular.z,
                m.pose.covariance[0], m.pose.covariance[7]))
            passed = passed and finite and len(messages) >= 2 and 0 <= age <= 0.5
            passed = passed and max_gap <= 0.5 and max_step <= 0.02 and max_vx < 0.016
            print(f'{key}: count={len(messages)} delta_xy={delta:.6f}m '
                  f'max_abs_vx={max_vx:.6f}m/s last_age={age:.3f}s')
            print(f'{key}: max_pose_step={max_step:.6f}m max_header_gap={max_gap:.3f}s')
            print(f'{key}: first_pose_var_xy=({first.pose.covariance[0]}, {first.pose.covariance[7]}) '
                  f'last_pose_var_xy=({last.pose.covariance[0]}, {last.pose.covariance[7]})')
            if key == 'raw':
                stale = sum(m.twist.covariance[0] >= 1000 for m in messages)
                print('RAW_STALE_FEEDBACK_SAMPLES=', stale)
                passed = passed and stale == 0
            else:
                passed = passed and max(max(m.pose.covariance[0], m.pose.covariance[7])
                                        for m in messages) <= 100
        print(f'IDLE_OBSERVATION_OK={passed}', flush=True)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0 if passed else 2


if __name__ == '__main__':
    raise SystemExit(main())
