#!/usr/bin/env python3

import math
import serial

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu


class IAHRSNode(Node):

    def __init__(self):
        super().__init__('iahrs_node')

        # Parameters
        self.declare_parameter('port', '/dev/ttyUSB2')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('frame_id', 'imu_link')
        self.declare_parameter('rate', 20.0)

        self.port = self.get_parameter('port').value
        self.baudrate = self.get_parameter('baudrate').value
        self.frame_id = self.get_parameter('frame_id').value
        self.rate = self.get_parameter('rate').value

        # ROS2 publisher
        self.publisher_ = self.create_publisher(
            Imu,
            '/imu/data',
            10
        )

        # Serial connection
        try:
            self.ser = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=0.1
            )

            self.ser.reset_input_buffer()

            self.get_logger().info(
                f'iAHRS connected: {self.port} @ {self.baudrate}'
            )

        except Exception as e:
            self.get_logger().error(
                f'Failed to open iAHRS serial port: {e}'
            )
            raise

        # Timer
        period = 1.0 / float(self.rate)
        self.timer = self.create_timer(period, self.timer_callback)

    def send_command(self, command):
        try:
            self.ser.reset_input_buffer()

            self.ser.write((command + '\n').encode('ascii'))
            self.ser.flush()

            response = self.ser.readline().decode(
                'ascii',
                errors='ignore'
            ).strip()

            # Expected:
            # g=x,y,z
            # a=x,y,z
            # e=roll,pitch,yaw
            if not response.startswith(command + '='):
                return None

            values = response.split('=', 1)[1].split(',')

            if len(values) != 3:
                return None

            return [float(v) for v in values]

        except Exception as e:
            self.get_logger().warn(
                f'Serial communication error: {e}'
            )
            return None

    @staticmethod
    def euler_to_quaternion(roll, pitch, yaw):
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)

        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)

        qx = sr * cp * cy - cr * sp * sy
        qy = cr * sp * cy + sr * cp * sy
        qz = cr * cp * sy - sr * sp * cy
        qw = cr * cp * cy + sr * sp * sy

        return qx, qy, qz, qw

    def timer_callback(self):

        gyro = self.send_command('g')
        accel = self.send_command('a')
        euler = self.send_command('e')

        if gyro is None or accel is None or euler is None:
            return

        # --------------------------------------------------
        # Gyro: deg/s -> rad/s
        # --------------------------------------------------
        gx = math.radians(gyro[0])
        gy = math.radians(gyro[1])
        gz = math.radians(gyro[2]) * 1.027

        # --------------------------------------------------
        # Acceleration: g -> m/s^2
        # --------------------------------------------------
        gravity = 9.80665

        ax = accel[0] * gravity
        ay = accel[1] * gravity
        az = accel[2] * gravity

        # --------------------------------------------------
        # Euler angle: degree -> radian
        # --------------------------------------------------
        roll = math.radians(euler[0])
        pitch = math.radians(euler[1])
        yaw = math.radians(euler[2])

        qx, qy, qz, qw = self.euler_to_quaternion(
            roll,
            pitch,
            yaw
        )

        # --------------------------------------------------
        # IMU message
        # --------------------------------------------------
        msg = Imu()

        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id

        # Orientation
        msg.orientation.x = qx
        msg.orientation.y = qy
        msg.orientation.z = qz
        msg.orientation.w = qw

        # Angular velocity
        # Gyro Z deadband
        # Suppress small stationary bias so EKF does not
        # continuously rotate while the robot is stopped.
        if abs(gz) < 0.02:
            gz = 0.0

        msg.angular_velocity.x = gx
        msg.angular_velocity.y = gy
        msg.angular_velocity.z = gz

        # Linear acceleration
        msg.linear_acceleration.x = ax
        msg.linear_acceleration.y = ay
        msg.linear_acceleration.z = az

        # Covariance
        # Initial practical values for EKF use.
        # These can be tuned later from stationary measurements.
        msg.orientation_covariance = [
            0.02, 0.0, 0.0,
            0.0, 0.02, 0.0,
            0.0, 0.0, 0.05
        ]

        msg.angular_velocity_covariance = [
            0.01, 0.0, 0.0,
            0.0, 0.01, 0.0,
            0.0, 0.0, 0.01
        ]

        msg.linear_acceleration_covariance = [
            0.10, 0.0, 0.0,
            0.0, 0.10, 0.0,
            0.0, 0.0, 0.10
        ]

        self.publisher_.publish(msg)

    def destroy_node(self):
        if hasattr(self, 'ser') and self.ser.is_open:
            self.ser.close()

        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    node = IAHRSNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
