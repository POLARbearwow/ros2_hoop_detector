# -*- coding: utf-8 -*-
"""
hoop_system.launch.py

启动一个组件容器，并在其中加载 HikCamera 组件和 HoopDetector 组件。
执行：
  ros2 launch hoop_detector hoop_system.launch.py
"""

from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description() -> LaunchDescription:
    # 组件容器
    container = ComposableNodeContainer(
        name="component_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container",
        output="screen",
        composable_node_descriptions=[
            # Hikvision 相机组件
            ComposableNode(
                package="hik_camera",
                plugin="hik_camera::HikCameraNode",
                name="hik_camera_node",
                extra_arguments=[{"use_intra_process_comms": True}],
            ),
            # HoopDetector 组件
            ComposableNode(
                package="hoop_detector",
                plugin="hoop_detector::HoopDetectorNode",
                name="hoop_detector_node",
                extra_arguments=[{"use_intra_process_comms": True}],
            ),
        ],
    )

    return LaunchDescription([container]) 