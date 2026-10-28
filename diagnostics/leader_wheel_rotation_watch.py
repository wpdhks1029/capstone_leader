"""Observe wheel rotations inferred from /odom; publishes nothing.
Not an independent encoder measurement. Compare against a filmed wheel mark.
"""
import math
import time


def rotations(v, w, dt, radius=0.085, track=0.466):
    return ((v-w*track/2)*dt/(2*math.pi*radius),
            (v+w*track/2)*dt/(2*math.pi*radius))


def main():
    import rclpy
    from nav_msgs.msg import Odometry
    rclpy.init()
    node = rclpy.create_node('leader_wheel_rotation_watch')
    rows = []
    node.create_subscription(Odometry, '/odom', rows.append, 10)
    print('OBSERVING 40 seconds; NO commands published. Start the 40cm test now.', flush=True)
    end = time.monotonic()+40
    try:
        while rclpy.ok() and time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.1)
        total = [0.0, 0.0]
        gaps = []
        invalid = 0
        for a, b in zip(rows, rows[1:]):
            dt = b.header.stamp.sec-a.header.stamp.sec + (b.header.stamp.nanosec-a.header.stamp.nanosec)/1e9
            gaps.append(dt)
            v, w = b.twist.twist.linear.x, b.twist.twist.angular.z
            if (not 0 < dt <= 0.5 or not all(map(math.isfinite, (v, w)))
                    or b.twist.covariance[0] >= 1000):
                invalid += 1
                continue
            left, right = rotations(v, w, dt)
            total[0] += left
            total[1] += right
        print(f'MESSAGES={len(rows)} INVALID_INTERVALS={invalid} MAX_GAP={max(gaps, default=0):.3f}s')
        for name, rev in zip(('LEFT', 'RIGHT'), total):
            print(f'{name}: INFERRED_REVOLUTIONS={rev:.4f} ANGLE={rev*360:.1f}deg')
        valid = len(rows) > 2 and invalid == 0
        print(f'OBSERVATION_VALID={valid}; compare with physical wheel mark, not ground distance.')
        return 0 if valid else 2
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
