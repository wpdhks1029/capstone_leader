#!/usr/bin/env python3

import math
import struct
import serial
import time

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


RMID = 0xB7
TMID = 0xB8

PID_PNT_VEL_CMD = 0xCF
PID_PNT_MONITOR = 0xD8


def checksum(packet):
    return (-sum(packet)) & 0xFF


def make_velocity_packet(
    controller_id,
    m1_rpm,
    m2_rpm,
    request_feedback=True
):

    data = []

    # ------------------------------------------------
    # IMPORTANT
    # Keep command ENABLE = 1 even at 0 rpm.
    # Otherwise STOP does not work on this MD200T.
    # ------------------------------------------------

    data.append(1)
    data.extend(
        struct.pack(
            "<h",
            int(m1_rpm)
        )
    )

    data.append(1)
    data.extend(
        struct.pack(
            "<h",
            int(m2_rpm)
        )
    )

    # ------------------------------------------------
    # D6 Return Data Type
    #
    # 0 = no return
    # 1 = PID_PNT_MONITOR (216)
    # ------------------------------------------------

    if request_feedback:
        data.append(1)
    else:
        data.append(0)

    packet = [
        RMID,
        TMID,
        controller_id,
        PID_PNT_VEL_CMD,
        0x07,
        *data
    ]

    packet.append(
        checksum(packet)
    )

    return bytes(packet)


class MD200TController(Node):

    def __init__(self):

        super().__init__(
            'md200t_controller'
        )

        # ============================================
        # Parameters
        # ============================================

        self.declare_parameter(
            'wheel_radius',
            0.085
        )

        self.declare_parameter(
            'wheel_track',
            0.560
        )

        self.declare_parameter(
            'max_rpm',
            100
        )

        self.declare_parameter(
            'invert_left',
            False
        )

        self.declare_parameter(
            'invert_right',
            False
        )

        self.declare_parameter(
            'cmd_vel_timeout',
            0.5
        )

        self.declare_parameter(
            'serial_port',
            '/dev/ttyUSB0'
        )

        self.declare_parameter(
            'baudrate',
            57600
        )

        self.declare_parameter(
            'controller_id',
            1
        )

        self.declare_parameter(
            'dry_run',
            True
        )

        self.declare_parameter(
            'allow_motion',
            False
        )

        # ============================================
        # Read parameters
        # ============================================

        self.declare_parameter('odom_frame_id', 'odom')
        self.declare_parameter('base_frame_id', 'base_link')

        self.odom_frame_id = self.get_parameter('odom_frame_id').value
        self.base_frame_id = self.get_parameter('base_frame_id').value

        self.wheel_radius = (
            self.get_parameter(
                'wheel_radius'
            ).value
        )

        self.wheel_track = (
            self.get_parameter(
                'wheel_track'
            ).value
        )

        self.max_rpm = (
            self.get_parameter(
                'max_rpm'
            ).value
        )

        self.invert_left = (
            self.get_parameter(
                'invert_left'
            ).value
        )

        self.invert_right = (
            self.get_parameter(
                'invert_right'
            ).value
        )

        self.cmd_vel_timeout = (
            self.get_parameter(
                'cmd_vel_timeout'
            ).value
        )

        self.serial_port = (
            self.get_parameter(
                'serial_port'
            ).value
        )

        self.baudrate = (
            self.get_parameter(
                'baudrate'
            ).value
        )

        self.controller_id = (
            self.get_parameter(
                'controller_id'
            ).value
        )

        self.dry_run = (
            self.get_parameter(
                'dry_run'
            ).value
        )

        self.allow_motion = (
            self.get_parameter(
                'allow_motion'
            ).value
        )

        # ============================================
        # State
        # ============================================

        self.last_cmd_time = None

        # 실제 RPM 피드백 유효성 검사
        self.last_feedback_time = None
        self.feedback_timeout = 0.3
        self.watchdog_active = True

        self.ser = None

        # Actual wheel feedback
        self.actual_m1_rpm = 0
        self.actual_m2_rpm = 0

        # Odometry state
        self.odom_x = 0.0
        self.odom_y = 0.0
        self.odom_yaw = 0.0

        self.last_odom_time = (
            self.get_clock().now()
        )

        # ============================================
        # Serial
        # ============================================

        if not self.dry_run:

            try:

                self.ser = serial.Serial(
                    port=self.serial_port,
                    baudrate=self.baudrate,
                    bytesize=serial.EIGHTBITS,
                    parity=serial.PARITY_NONE,
                    stopbits=serial.STOPBITS_ONE,
                    timeout=0.03,
                    write_timeout=0.5
                )

                self.get_logger().info(
                    f'SERIAL OPEN SUCCESS: '
                    f'{self.serial_port} '
                    f'@ {self.baudrate}'
                )

            except serial.SerialException as e:

                self.get_logger().error(
                    f'SERIAL OPEN FAILED: {e}'
                )

                raise

        # ============================================
        # ROS interfaces
        # ============================================

        self.subscription = (
            self.create_subscription(
                Twist,
                '/cmd_vel',
                self.cmd_vel_callback,
                10
            )
        )

        self.odom_pub = (
            self.create_publisher(
                Odometry,
                '/odom',
                20
            )
        )

        # Watchdog
        self.watchdog_timer = (
            self.create_timer(
                0.1,
                self.watchdog_callback
            )
        )

        # Odom publish/integration timer
        self.odom_timer = (
            self.create_timer(
                0.05,
                self.odom_callback
            )
        )

        # ============================================
        # Startup info
        # ============================================

        self.get_logger().info(
            '================================'
        )

        self.get_logger().info(
            'Leader Robot MD200T Controller'
        )

        self.get_logger().info(
            '================================'
        )

        self.get_logger().info(
            f'DRY_RUN = {self.dry_run}'
        )

        self.get_logger().info(
            f'ALLOW_MOTION = '
            f'{self.allow_motion}'
        )

        self.get_logger().info(
            f'wheel_radius = '
            f'{self.wheel_radius:.3f} m'
        )

        self.get_logger().info(
            f'wheel_track = '
            f'{self.wheel_track:.3f} m'
        )

        self.get_logger().info(
            f'max_rpm = {self.max_rpm}'
        )

        self.get_logger().info(
            f'cmd_vel_timeout = '
            f'{self.cmd_vel_timeout:.2f} sec'
        )

        self.get_logger().info(
            f'invert_left = '
            f'{self.invert_left}'
        )

        self.get_logger().info(
            f'invert_right = '
            f'{self.invert_right}'
        )

        self.get_logger().info(
            f'serial_port = '
            f'{self.serial_port}'
        )

        self.get_logger().info(
            f'baudrate = '
            f'{self.baudrate}'
        )

        self.get_logger().info(
            f'controller_id = '
            f'{self.controller_id}'
        )

        if self.dry_run:

            self.get_logger().info(
                'SERIAL TRANSMISSION DISABLED'
            )

        else:

            self.get_logger().info(
                'SERIAL TRANSMISSION ENABLED'
            )

        self.get_logger().info(
            'REAL RPM FEEDBACK ENABLED'
        )

        self.get_logger().info(
            'REAL ODOMETRY ENABLED'
        )

        self.get_logger().info(
            '================================'
        )

        # Confirmed STOP
        self.send_packet(
            'STARTUP STOP',
            0,
            0
        )


    # ================================================
    # cmd_vel -> requested RPM
    # ================================================

    def calculate_rpm(
        self,
        linear_x,
        angular_z
    ):

        v_left = (
            linear_x
            - angular_z
            * self.wheel_track
            / 2.0
        )

        v_right = (
            linear_x
            + angular_z
            * self.wheel_track
            / 2.0
        )

        left_rpm = (
            v_left
            / (
                2.0
                * math.pi
                * self.wheel_radius
            )
            * 60.0
        )

        right_rpm = (
            v_right
            / (
                2.0
                * math.pi
                * self.wheel_radius
            )
            * 60.0
        )

        left_rpm = max(
            -self.max_rpm,
            min(
                self.max_rpm,
                left_rpm
            )
        )

        right_rpm = max(
            -self.max_rpm,
            min(
                self.max_rpm,
                right_rpm
            )
        )

        m1 = int(
            round(left_rpm)
        )

        m2 = int(
            round(right_rpm)
        )

        if self.invert_left:
            m1 = -m1

        if self.invert_right:
            m2 = -m2

        return m1, m2


    # ================================================
    # Read one MD200T response frame
    # ================================================

    def read_feedback_frame(
        self,
        timeout=0.05
    ):

        if self.ser is None:
            return None

        end_time = (
            time.time()
            + timeout
        )

        rx = bytearray()

        while time.time() < end_time:

            waiting = (
                self.ser.in_waiting
            )

            if waiting:

                rx.extend(
                    self.ser.read(
                        waiting
                    )
                )

            # PID216 total frame = 20 bytes
            if len(rx) >= 20:
                break

            time.sleep(
                0.001
            )

        if not rx:
            return None

        # Search valid response frame:
        # B8 B7 ID D8 0E ... checksum
        for i in range(
            max(
                0,
                len(rx) - 19
            )
            + 1
        ):

            if (
                i + 20
                > len(rx)
            ):
                break

            if (
                rx[i] == TMID
                and rx[i + 1] == RMID
                and rx[i + 2]
                    == self.controller_id
                and rx[i + 3]
                    == PID_PNT_MONITOR
                and rx[i + 4]
                    == 0x0E
            ):

                frame = bytes(
                    rx[i:i + 20]
                )

                if (
                    sum(frame)
                    & 0xFF
                ) == 0:

                    return frame

        return None


    # ================================================
    # Parse PID216
    # ================================================

    def parse_feedback(
        self,
        frame
    ):

        if frame is None:
            return False

        data = frame[5:19]

        if len(data) != 14:
            return False

        # Experimentally verified:
        #
        # data[0:2] = M1 actual RPM
        # data[7:9] = M2 actual RPM

        m1_rpm = (
            struct.unpack(
                '<h',
                data[0:2]
            )[0]
        )

        m2_rpm = (
            struct.unpack(
                '<h',
                data[7:9]
            )[0]
        )

        self.actual_m1_rpm = (
            m1_rpm
        )

        self.actual_m2_rpm = (
            m2_rpm
        )

        self.last_feedback_time = self.get_clock().now()

        return True


    # ================================================
    # TX + RX
    # ================================================

    def send_packet(
        self,
        title,
        m1,
        m2
    ):

        if (
            not self.allow_motion
            and (
                m1 != 0
                or m2 != 0
            )
        ):

            self.get_logger().warning(
                f'MOTION BLOCKED: '
                f'M1={m1}, '
                f'M2={m2} '
                f'-> forcing STOP'
            )

            m1 = 0
            m2 = 0

        packet = (
            make_velocity_packet(
                self.controller_id,
                m1,
                m2,
                request_feedback=True
            )
        )

        checksum_ok = (
            sum(packet)
            & 0xFF
        ) == 0

        self.get_logger().info(
            f'{title}: '
            f'M1={m1} rpm, '
            f'M2={m2} rpm'
        )

        self.get_logger().info(
            'PACKET = '
            + packet.hex(' ')
        )

        self.get_logger().info(
            f'CHECKSUM = '
            f'{checksum_ok}'
        )

        if not checksum_ok:

            self.get_logger().error(
                'CHECKSUM ERROR - '
                'packet NOT transmitted'
            )

            return

        if self.dry_run:

            self.get_logger().info(
                'DRY-RUN: '
                'packet NOT transmitted'
            )

            return

        if self.ser is None:

            self.get_logger().error(
                'SERIAL NOT OPEN'
            )

            return

        try:

            # Clear previous response.
            self.ser.reset_input_buffer()

            written = (
                self.ser.write(
                    packet
                )
            )

            self.ser.flush()

            frame = (
                self.read_feedback_frame(
                    timeout=0.05
                )
            )

            if self.parse_feedback(
                frame
            ):

                self.get_logger().info(
                    f'RX FEEDBACK: '
                    f'M1={self.actual_m1_rpm} rpm, '
                    f'M2={self.actual_m2_rpm} rpm'
                )

            else:

                self.get_logger().warning(
                    'RX FEEDBACK: NONE'
                )

            self.get_logger().info(
                f'TX SUCCESS: '
                f'{written} bytes'
            )

        except serial.SerialException as e:

            self.get_logger().error(
                f'SERIAL TX/RX ERROR: '
                f'{e}'
            )


    # ================================================
    # cmd_vel callback
    # ================================================

    def cmd_vel_callback(
        self,
        msg
    ):

        self.last_cmd_time = (
            self.get_clock().now()
        )

        if self.watchdog_active:

            self.watchdog_active = (
                False
            )

            self.get_logger().info(
                'WATCHDOG cleared: '
                'cmd_vel received'
            )

        m1, m2 = (
            self.calculate_rpm(
                msg.linear.x,
                msg.angular.z
            )
        )

        self.get_logger().info(
            '-----------------------------'
        )

        self.get_logger().info(
            f'cmd_vel: '
            f'linear='
            f'{msg.linear.x:.3f} m/s, '
            f'angular='
            f'{msg.angular.z:.3f} rad/s'
        )

        self.send_packet(
            'COMMAND',
            m1,
            m2
        )


    # ================================================
    # Watchdog
    # ================================================

    def watchdog_callback(
        self
    ):

        if self.last_cmd_time is None:
            return

        now = (
            self.get_clock().now()
        )

        elapsed = (
            now
            - self.last_cmd_time
        ).nanoseconds / 1e9

        if (
            elapsed
            > self.cmd_vel_timeout
            and not self.watchdog_active
        ):

            self.watchdog_active = (
                True
            )

            self.get_logger().warning(
                f'WATCHDOG: '
                f'no cmd_vel for '
                f'{elapsed:.2f} sec'
            )

            # Send repeated STOP commands so that the
            # final feedback reaches actual 0 rpm.
            for i in range(5):

                self.send_packet(
                    f'WATCHDOG STOP {i + 1}/5',
                    0,
                    0
                )

                time.sleep(0.05)

            # After confirmed stop sequence, prevent
            # stale residual RPM from being integrated.
            if (
                abs(self.actual_m1_rpm) <= 2
                and abs(self.actual_m2_rpm) <= 2
            ):
                self.actual_m1_rpm = 0
                self.actual_m2_rpm = 0


    # ================================================
    # Actual RPM -> odometry
    # ================================================

    def odom_callback(
        self
    ):

        now = (
            self.get_clock().now()
        )

        dt = (
            now
            - self.last_odom_time
        ).nanoseconds / 1e9

        self.last_odom_time = now

        if (
            dt <= 0.0
            or dt > 1.0
        ):
            return

        # ------------------------------------------------
        # Physical wheel directions
        #
        # These are already expected to represent
        # robot-forward-positive RPM.
        # ------------------------------------------------

        left_rpm = (
            float(
                self.actual_m1_rpm
            )
        )

        right_rpm = (
            float(
                self.actual_m2_rpm
            )
        )

        # If feedback sign follows motor electrical
        # direction instead of robot frame, preserve
        # configured inversions here too.
        if self.invert_left:
            left_rpm = -left_rpm

        if self.invert_right:
            right_rpm = -right_rpm

        # 최신 RPM 피드백이 없으면 위치 적분 금지
        feedback_valid = (
            self.last_feedback_time is not None
            and (
                now - self.last_feedback_time
            ).nanoseconds / 1e9 <= self.feedback_timeout
        )

        if not feedback_valid:
            left_rpm = 0.0
            right_rpm = 0.0

            self.get_logger().warning(
                'RPM FEEDBACK STALE - ODOM INTEGRATION DISABLED',
                throttle_duration_sec=2.0
            )

        # RPM -> rad/s
        left_omega = (
            left_rpm
            * 2.0
            * math.pi
            / 60.0
        )

        right_omega = (
            right_rpm
            * 2.0
            * math.pi
            / 60.0
        )

        # Wheel linear velocities
        v_left = (
            self.wheel_radius
            * left_omega
        )

        v_right = (
            self.wheel_radius
            * right_omega
        )

        # Differential-drive kinematics
        linear_v = (
            v_left
            + v_right
        ) / 2.0

        angular_v = (
            v_right
            - v_left
        ) / self.wheel_track

        # Midpoint integration gives better
        # behaviour while turning.
        yaw_mid = (
            self.odom_yaw
            + angular_v
            * dt
            / 2.0
        )

        self.odom_x += (
            linear_v
            * math.cos(
                yaw_mid
            )
            * dt
        )

        self.odom_y += (
            linear_v
            * math.sin(
                yaw_mid
            )
            * dt
        )

        self.odom_yaw += (
            angular_v
            * dt
        )

        self.odom_yaw = (
            math.atan2(
                math.sin(
                    self.odom_yaw
                ),
                math.cos(
                    self.odom_yaw
                )
            )
        )

        qz = (
            math.sin(
                self.odom_yaw
                / 2.0
            )
        )

        qw = (
            math.cos(
                self.odom_yaw
                / 2.0
            )
        )

        # ============================================
        # /odom
        # ============================================

        odom = Odometry()

        odom.header.stamp = (
            now.to_msg()
        )

        odom.header.frame_id = self.odom_frame_id

        odom.child_frame_id = self.base_frame_id

        odom.pose.pose.position.x = (
            self.odom_x
        )

        odom.pose.pose.position.y = (
            self.odom_y
        )

        odom.pose.pose.position.z = 0.0

        odom.pose.pose.orientation.x = 0.0
        odom.pose.pose.orientation.y = 0.0
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw

        odom.twist.twist.linear.x = (
            linear_v
        )

        odom.twist.twist.angular.z = (
            angular_v
        )

        if not feedback_valid:
            # 실제 정지 여부는 알 수 없으므로 큰 불확실성 표시
            odom.twist.covariance[0] = 1e6
            odom.twist.covariance[5] = 1e6
            odom.twist.covariance[35] = 1e6

        self.odom_pub.publish(
            odom
        )



    # ================================================
    # Shutdown
    # ================================================

    def safe_shutdown(
        self
    ):

        self.get_logger().info(
            'SAFE SHUTDOWN'
        )

        try:

            # Multiple STOP packets for safety
            for _ in range(3):

                self.send_packet(
                    'SHUTDOWN STOP',
                    0,
                    0
                )

                time.sleep(
                    0.05
                )

        except Exception as e:

            self.get_logger().error(
                f'STOP ERROR: {e}'
            )

        if self.ser is not None:

            try:

                self.ser.close()

                self.get_logger().info(
                    'SERIAL CLOSED'
                )

            except serial.SerialException as e:

                self.get_logger().error(
                    f'SERIAL CLOSE ERROR: '
                    f'{e}'
                )


def main(args=None):

    rclpy.init(
        args=args
    )

    node = (
        MD200TController()
    )

    try:

        rclpy.spin(
            node
        )

    except KeyboardInterrupt:

        pass

    finally:

        node.safe_shutdown()

        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
