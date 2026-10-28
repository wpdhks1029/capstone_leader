"""Diagnostic PID4 polling wrapper; imports but never edits the leader controller.

One serial owner, synchronous transactions, explicitly single-threaded executor.
Periodic polling sends only PID4(PID216). Startup/shutdown and motion commands
remain the existing controller's behavior. Use the dedicated diagnostic launch.
"""
import math
import signal
import subprocess
import sys

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.signals import SignalHandlerOptions
import serial

from leader_robot_control.md200t_controller import MD200TController


PORT = '/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_AP2472IE-if00-port0'


def make_feedback_request(controller_id):
    if not 1 <= controller_id <= 255:
        raise ValueError('Invalid controller ID')
    body = bytes((0xB7, 0xB8, controller_id, 0x04, 0x01, 0xD8))
    return body + bytes([(-sum(body)) & 0xff])


class LateralConstraintPublisher:
    """Annotate the baseline's zero lateral velocity as a model constraint.

This is not a lateral-velocity sensor measurement. Forward velocity and its
valid/stale covariance still come from the inherited wheel feedback logic.
"""
    def __init__(self, publisher, variance):
        if not math.isfinite(variance) or variance <= 0:
            raise ValueError('Lateral constraint variance must be finite and positive')
        self.publisher = publisher
        self.variance = variance

    def publish(self, msg):
        if msg.twist.twist.linear.y != 0.0:
            raise ValueError('Cannot replace nonzero lateral velocity with a model constraint')
        msg.twist.covariance[7] = self.variance
        self.publisher.publish(msg)


class PollingMD200TController(MD200TController):
    def __init__(self):
        super().__init__()
        try:
            self.declare_parameter('feedback_speed_scale', 1.0)
            self.feedback_speed_scale = float(self.get_parameter('feedback_speed_scale').value)
            if not math.isfinite(self.feedback_speed_scale) or not 0.8 <= self.feedback_speed_scale <= 1.3:
                raise ValueError('feedback_speed_scale must be finite and 0.8..1.3')
            self.declare_parameter('constrain_lateral_velocity', False)
            self.declare_parameter('lateral_velocity_variance', 0.0004)
            if self.get_parameter('constrain_lateral_velocity').value:
                variance = float(self.get_parameter('lateral_velocity_variance').value)
                self.odom_pub = LateralConstraintPublisher(self.odom_pub, variance)
            self.declare_parameter('feedback_poll_rate', 10.0)
            rate = float(self.get_parameter('feedback_poll_rate').value)
            if not math.isfinite(rate) or not 5 <= rate <= 10:
                raise ValueError('Diagnostic feedback_poll_rate must be 5..10Hz')
            self.poll_requests = self.poll_valid = self.poll_missing = self.poll_errors = 0
            self.poll_timer = self.create_timer(1.0 / rate, self.poll_feedback)
        except Exception:
            super().safe_shutdown()
            self.destroy_node()
            raise
        self.get_logger().info(
            f'Diagnostic polling enabled at {rate}Hz; same serial connection, PID4 only')

    def odom_callback(self):
        # SingleThreadedExecutor: feedback cannot change during this callback.
        # Scale only the RPM-to-odometry conversion. Preserve raw feedback,
        # physical radius and outgoing command conversion, including on failure.
        scale = getattr(self, 'feedback_speed_scale', 1.0)
        left, right = self.actual_m1_rpm, self.actual_m2_rpm
        try:
            self.actual_m1_rpm = left * scale
            self.actual_m2_rpm = right * scale
            return super().odom_callback()
        finally:
            self.actual_m1_rpm, self.actual_m2_rpm = left, right

    def poll_feedback(self):
        if self.dry_run or self.ser is None:
            return
        request = make_feedback_request(self.controller_id)
        try:
            self.ser.reset_input_buffer()
            written = self.ser.write(request)
            self.poll_requests += 1
            if written != len(request):
                raise serial.SerialException(f'Partial PID4 write: {written}/{len(request)}')
            self.ser.flush()
            frame = self.read_feedback_frame(timeout=0.05)
            if self.parse_feedback(frame):
                self.poll_valid += 1
                self.get_logger().info(
                    f'PID4 RX M1={self.actual_m1_rpm} M2={self.actual_m2_rpm} rpm; '
                    f'POLL_TX={self.poll_requests} VALID={self.poll_valid} '
                    f'MISSING={self.poll_missing} ERRORS={self.poll_errors}',
                    throttle_duration_sec=2.0,
                )
            else:
                # Preserve last_feedback_time. The existing stale-data gate
                # must notice missing feedback; never invent a valid zero RPM.
                self.poll_missing += 1
                self.get_logger().warning(
                    f'PID4 no valid response: TX={self.poll_requests} '
                    f'VALID={self.poll_valid} MISSING={self.poll_missing}',
                    throttle_duration_sec=2.0,
                )
        except (serial.SerialException, OSError) as exc:
            self.poll_errors += 1
            self.get_logger().error(f'PID4 transaction failed: {exc}', throttle_duration_sec=2.0)

    def safe_shutdown(self):
        if hasattr(self, 'poll_timer'):
            self.poll_timer.cancel()
        super().safe_shutdown()


def main():
    if '--offline-check' in sys.argv[1:]:
        print('OFFLINE ONLY; request:', make_feedback_request(1).hex(' '))
        return 0
    # Fail closed if the known leader serial port is already in use.
    owner = subprocess.run(['fuser', PORT], capture_output=True, text=True)
    if owner.returncode != 1 or owner.stderr.strip():
        raise SystemExit(f'Serial port not confirmed free: {owner.stdout} {owner.stderr}')
    def interrupt(signum, frame):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, interrupt)
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node, executor = None, None
    try:
        node = PollingMD200TController()
        executor = SingleThreadedExecutor()
        executor.add_node(node)
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.safe_shutdown()
            node.destroy_node()
        if executor is not None:
            executor.shutdown()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
