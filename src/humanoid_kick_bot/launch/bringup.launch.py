#!/usr/bin/env python3
"""
bringup.launch.py
------------------
Brings up the full modular pipeline described in the assignment:

    perception (ball_detector_node)
        -> planning (ik_planner_node: TF transform + FK/IK + state machine)
        -> control  (servo_controller_node: PWM + serial to ESP32)

plus robot_state_publisher (URDF -> TF tree) and RViz2 for visualization.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    pkg_share = get_package_share_directory('humanoid_kick_bot')
    urdf_path = os.path.join(pkg_share, 'urdf', 'humanoid_leg.urdf.xacro')
    config_path = os.path.join(pkg_share, 'config', 'servo_config.yaml')
    rviz_path = os.path.join(pkg_share, 'rviz', 'humanoid.rviz')

    camera_source_arg = DeclareLaunchArgument(
        'camera_source', default_value='0',
        description='Fallback if esp32_cam_ip is not set: webcam index (e.g. 0) or a full MJPEG URL')
    esp32_cam_ip_arg = DeclareLaunchArgument(
        'esp32_cam_ip', default_value='',
        description='ESP32-CAM IP address, e.g. 192.168.1.50 - preferred way to point at the camera; '
                    'builds http://<ip>:81/stream automatically. Leave empty to use camera_source instead.')
    serial_port_arg = DeclareLaunchArgument(
        'serial_port', default_value='/dev/ttyUSB0',
        description='Serial port to the ESP32 running esp32_pca9685_bridge.ino')
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz', default_value='true', description='Launch RViz2')

    robot_description = ParameterValue(
        Command(['xacro ', urdf_path]),
        value_type=str
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}],
    )

    ball_detector = Node(
        package='humanoid_kick_bot',
        executable='ball_detector_node',
        name='ball_detector_node',
        output='screen',
        parameters=[{
            'camera_source': LaunchConfiguration('camera_source'),
            'esp32_cam_ip': LaunchConfiguration('esp32_cam_ip'),
        }],
    )

    ik_planner = Node(
        package='humanoid_kick_bot',
        executable='ik_planner_node',
        name='ik_planner_node',
        output='screen',
        parameters=[{'config_path': config_path}],
    )

    servo_controller = Node(
        package='humanoid_kick_bot',
        executable='servo_controller_node',
        name='servo_controller_node',
        output='screen',
        parameters=[{
            'config_path': config_path,
            'serial_port': ParameterValue(
                LaunchConfiguration('serial_port'),
                value_type=str
            ),
        }],
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_path],
        condition=IfCondition(LaunchConfiguration('use_rviz')),
    )

    return LaunchDescription([
        camera_source_arg,
        esp32_cam_ip_arg,
        serial_port_arg,
        use_rviz_arg,
        robot_state_publisher,
        ball_detector,
        ik_planner,
        servo_controller,
        rviz,
    ])
