import os

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import xacro


def generate_launch_description():

    # =========================================================
    # Robot URDF / Xacro
    # =========================================================
    package_share = get_package_share_directory(
        'leader_robot_control'
    )

    xacro_file = os.path.join(
        package_share,
        'urdf',
        'leader_robot.urdf.xacro'
    )

    robot_description = xacro.process_file(
        xacro_file
    ).toxml()


    # =========================================================
    # Device fixed paths
    # =========================================================

    # SLLIDAR A2M12
    lidar_port = (
        '/dev/serial/by-id/'
        'usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_'
        'e2085ebc65c7654c9a900fe6a5f7c902-if00-port0'
    )

    # iAHRS
    iahrs_port = (
        '/dev/serial/by-id/'
        'usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_'
        'Controller_0001-if00-port0'
    )

    # MD200T
    md200t_port = (
        '/dev/serial/by-id/'
        'usb-FTDI_FT232R_USB_UART_AP2472IE-if00-port0'
    )


    return LaunchDescription([

        # =====================================================
        # Robot State Publisher
        # =====================================================
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',

            parameters=[{
                'robot_description': robot_description
            }]
        ),


        # =====================================================
        # SLLIDAR A2M12
        # =====================================================
        Node(
            package='sllidar_ros2',
            executable='sllidar_node',
            name='sllidar_node',
            output='screen',

            parameters=[{
                'channel_type': 'serial',
                'serial_port': lidar_port,
                'serial_baudrate': 256000,
                'frame_id': 'laser',
                'inverted': False,
                'angle_compensate': True,
            }]
        ),


        # =====================================================
        # iAHRS
        # =====================================================
        Node(
            package='iahrs_ros2',
            executable='iahrs_node',
            name='iahrs_node',
            output='screen',

            parameters=[{
                'port': iahrs_port,
                'baudrate': 115200,
            }]
        ),


        # =====================================================
        # MD200T Motor Controller
        # =====================================================
        Node(
            package='leader_robot_control',
            executable='md200t_controller',
            name='md200t_controller',
            output='screen',

            parameters=[{
                'dry_run': False,
                'allow_motion': True,

                'serial_port': md200t_port,
                'baudrate': 57600,

                'controller_id': 1,

                'wheel_radius': 0.085,
                'wheel_track': 0.466,

                'max_rpm': 100,
                'cmd_vel_timeout': 0.5,

                'invert_left': False,
                'invert_right': False,
            }]
        ),

    ])
