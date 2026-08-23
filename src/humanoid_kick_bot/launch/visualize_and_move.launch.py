#!/usr/bin/env python3
"""
visualize_and_move.launch.py
------------------------------
Dev/testing launch file: brings up ONLY robot_state_publisher + RViz2 +
servo_controller_node. No camera, no ball detection, no IK planner.

Use this while manually testing joint motion via:
    ros2 topic pub --once /joint_commands sensor_msgs/msg/JointState "{...}"

RViz will mirror every command you publish, since servo_controller_node
republishes whatever it sends to hardware back out on /joint_states.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('humanoid_kick_bot')
    urdf_path = os.path.join(pkg_share, 'urdf', 'humanoid_leg.urdf.xacro')
    config_path = os.path.join(pkg_share, 'config', 'servo_config.yaml')
    rviz_path = os.path.join(pkg_share, 'rviz', 'humanoid.rviz')

    serial_port_arg = DeclareLaunchArgument(
        'serial_port', default_value='/dev/ttyUSB0',
        description='Serial port to the ESP32 running esp32_pca9685_bridge.ino')

    robot_description = Command(['xacro ', urdf_path])

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}],
    )

    servo_controller = Node(
        package='humanoid_kick_bot',
        executable='servo_controller_node',
        name='servo_controller_node',
        output='screen',
        parameters=[{
            'config_path': config_path,
            'serial_port': LaunchConfiguration('serial_port'),
        }],
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_path],
    )

    return LaunchDescription([
        serial_port_arg,
        robot_state_publisher,
        servo_controller,
        rviz,
    ])
