"""Leader sensor check only: no MD200T, EKF, Nav2, or velocity publisher.

Run only after sensor communication is authorized. LiDAR scanning rotates its
internal scanner; IMU acquisition sends serial queries.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import xacro


def generate_launch_description():
    robot_description = xacro.process_file(os.path.join(
        get_package_share_directory('leader_robot_control'),
        'urdf', 'leader_robot.urdf.xacro',
    )).toxml()

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': robot_description,
                'use_sim_time': False,
            }],
        ),
        Node(
            package='iahrs_ros2',
            executable='iahrs_node',
            name='iahrs_node',
            output='screen',
            parameters=[{
                'port': (
                    '/dev/serial/by-id/'
                    'usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_'
                    'Controller_0001-if00-port0'
                ),
                'baudrate': 115200,
                'frame_id': 'imu_link',
                'rate': 20.0,
            }],
        ),
        Node(
            package='sllidar_ros2',
            executable='sllidar_node',
            name='sllidar_node',
            output='screen',
            parameters=[{
                'channel_type': 'serial',
                'serial_port': (
                    '/dev/serial/by-id/'
                    'usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_'
                    'e2085ebc65c7654c9a900fe6a5f7c902-if00-port0'
                ),
                'serial_baudrate': 256000,
                'frame_id': 'laser',
                'inverted': False,
                'angle_compensate': True,
            }],
        ),
    ])
