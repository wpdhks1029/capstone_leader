"""Diagnostic polling with a finite-variance vy=0 model constraint.

Default motion gate is false. Uses a diagnostic EKF YAML, not the success YAML.
The variance is a trial value, not a measured sensor calibration.
"""
import os
import sys

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    root = os.path.dirname(__file__)
    return LaunchDescription([
        DeclareLaunchArgument('allow_motion', default_value='false'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(
            root, 'leader_sensors_check.launch.py',
        ))),
        Node(
            package='robot_localization', executable='ekf_node',
            name='ekf_filter_node', output='screen',
            parameters=[os.path.join(root, 'leader_polling_constrained_ekf.yaml'),
                        {'use_sim_time': False}],
        ),
        Node(
            executable=sys.executable,
            arguments=[os.path.join(root, 'md200t_polling_controller.py')],
            name='md200t_controller', output='screen',
            parameters=[{
                'dry_run': False,
                'allow_motion': ParameterValue(
                    LaunchConfiguration('allow_motion'), value_type=bool),
                'serial_port': (
                    '/dev/serial/by-id/'
                    'usb-FTDI_FT232R_USB_UART_AP2472IE-if00-port0'
                ),
                'baudrate': 57600, 'controller_id': 1,
                'wheel_radius': 0.085, 'wheel_track': 0.466,
                'max_rpm': 25, 'cmd_vel_timeout': 0.5,
                'invert_left': False, 'invert_right': False,
                'feedback_poll_rate': 10.0,
                'feedback_speed_scale': 1.18,
                'constrain_lateral_velocity': True,
                'lateral_velocity_variance': 0.0004,
            }],
        ),
    ])
