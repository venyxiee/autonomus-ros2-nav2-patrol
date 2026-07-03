from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():

    lidar_launch = os.path.join(
        get_package_share_directory('rplidar_ros'),
        'launch',
        'rplidar_c1_launch.py'
    )

    return LaunchDescription([

        # ======================
        # SERIAL
        # ======================
        Node(
            package='my_robot',
            executable='serial_bridge',
            name='serial_bridge',
            output='screen'
        ),

        # ======================
        # JOYSTICK
        # ======================
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen'
        ),

        # ======================
        # TELEOP
        # ======================
        Node(
            package='teleop_twist_joy',
            executable='teleop_node',
            name='teleop',
            output='screen',
            parameters=[{
                'require_enable_button': False,
                'axis_linear.x': 1,
                'axis_angular.yaw': 2,
                'scale_linear.x': 1.0,
                'scale_angular.yaw': 1.0,
                'deadzone': 0.2
            }]
        ),

        # ======================
        # ODOM
        # ======================
        Node(
            package='my_robot',
            executable='odom_node',
            name='odom_node',
            output='screen'
        ),

        # ======================
        # IMU
        # ======================
        Node(
            package='my_robot',
            executable='imu_node',
            name='imu_node',
            output='screen'
        ),

        # ======================
        # LIDAR
        # ======================
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(lidar_launch),
            launch_arguments={
                'serial_port': '/dev/ttyUSB1',
                'serial_baudrate': '460800',
                'frame_id': 'lidar_link'
            }.items()
        ),

        # ======================
        # TF LIDAR
        # LiDAR offset: 17cm depan, 5cm atas base_link
        # Rotasi 180° (3.14159) karena LiDAR terpasang terbalik
        # ======================
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0.17', '0', '0.05', '3.14159', '0', '0', 'base_link', 'lidar_link']
        ),

        # ======================
        # TF IMU
        # IMU di tengah robot (center of gravity = base_link)
        # ======================
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'imu_link']
        ),

        # ======================
        # AUTO INITIAL POSE PUBLISHER (Opsi B)
        # Publish /initialpose otomatis agar AMCL langsung punya
        # referensi tanpa perlu set manual di RViz.
        # Pastikan sudah didaftarkan di setup.py!
        # ======================
        Node(
            package='my_robot',
            executable='initial_pose_publisher',
            name='initial_pose_publisher',
            output='screen'
        ),

    ])
