"""Bounded left 45-degree diagnostic; default prints plan, publishes nothing."""
import argparse
import math
import signal
import time
import rclpy
from rclpy.signals import SignalHandlerOptions
from geometry_msgs.msg import Twist
from leader_ground_straight_check import GroundCheck, yaw, delta


def angle_delta(first, last):
    d = yaw(last)-yaw(first)
    return math.atan2(math.sin(d), math.cos(d))


class TurnCheck(GroundCheck):
    def __init__(self, node, direction='left'):
        if direction not in ('left', 'right'):
            raise ValueError('Invalid direction')
        self.direction = direction
        self.turn_sign = 1 if direction == 'left' else -1
        super().__init__(node)

    def publish_turn(self, speed):
        msg = Twist()
        msg.angular.z = speed
        self.pub.publish(msg)

    def turn_guard(self, first):
        self.guard()
        for key, initial in first.items():
            last = self.samples[key][-1]
            if math.hypot(*delta(initial, last)) > 0.05:
                raise RuntimeError(f'{key}: translation exceeds 5cm')
            a = self.turn_sign * angle_delta(initial, last)
            if not -0.08 <= a <= 1.0:
                raise RuntimeError(f'{key}: reverse rotation or angle exceeds limit')

    def run(self):
        first, reached, failed, stopped = None, False, False, False
        try:
            self.preflight()
            print('PRECHECK PASSED; zero-speed warmup', flush=True)
            self.warmup()
            self.guard()
            first = {k: rows[-1] for k, rows in self.samples.items()}
            print(f'{self.direction.upper()} 45deg; angular command={self.turn_sign*0.08:.2f}rad/s; timeout=25s', flush=True)
            end, next_pub = time.monotonic()+25, time.monotonic()
            while time.monotonic() < end:
                self.turn_guard(first)
                if self.turn_sign * angle_delta(first['raw'], self.samples['raw'][-1]) >= math.pi/4:
                    reached = True
                    break
                if time.monotonic() >= next_pub:
                    self.publish_turn(self.turn_sign * 0.08)
                    next_pub = time.monotonic()+0.1
                rclpy.spin_once(self.node, timeout_sec=0.01)
            if not reached:
                raise RuntimeError('45deg target not reached in 25s')
        except (RuntimeError, KeyboardInterrupt) as exc:
            failed = True
            print(f'TEST FAILED/INTERRUPTED: {exc}', flush=True)
        finally:
            print('ZERO-SPEED shutdown phase', flush=True)
            end = time.monotonic()+2
            while rclpy.ok() and time.monotonic() < end:
                self.publish_turn(0.0)
                rclpy.spin_once(self.node, timeout_sec=0.05)
            try:
                self.guard()
                if first is not None:
                    self.turn_guard(first)
                last = self.samples['raw'][-1]
                stopped = abs(last.twist.twist.linear.x) < 0.016 and abs(last.twist.twist.angular.z) < 0.04
            except RuntimeError as exc:
                failed = True
                print(f'FINAL STATE INVALID: {exc}', flush=True)
            print(f'FINAL_RAW_ODOM_NEAR_ZERO={stopped}', flush=True)
            if first is not None:
                for k, initial in first.items():
                    print(f'{k}: rotation={math.degrees(angle_delta(initial,self.samples[k][-1])):.2f}deg '
                          f'translation={math.hypot(*delta(initial,self.samples[k][-1])):.4f}m', flush=True)
            complete = reached and stopped and not failed
            print(f'TURN_DIAGNOSTIC_COMPLETED={complete}', flush=True)
            print('Measure physical angle separately; odometry is not ground truth.', flush=True)
        return 0 if complete else 2


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--run-ground-turn', action='store_true')
    p.add_argument('--direction', choices=('left', 'right'), default='left')
    args = p.parse_args()
    if not args.run_ground_turn:
        print(f'PLAN ONLY: {args.direction} 45deg, max25s, then zero. No ROS init.')
        return 0
    def interrupt(signum, frame):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, interrupt)
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = rclpy.create_node('leader_ground_turn_check')
    try:
        return TurnCheck(node, direction=args.direction).run()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
