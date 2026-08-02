"""Jetson USB 摄像头（icSpring UVC）入口，发布到 /adas/sensors/camera/*。

采集节点 /dev/video0，640x480 YUYV @30，节点内转 rgb8 发布 image_raw。
详见记忆 orin-usb-camera 与 SoC/docs/06_板端TensorRT感知接入计划.md。
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_config = os.path.join(
        get_package_share_directory('adas_launch'),
        'config', 'camera.yaml')
    config = LaunchConfiguration('params_file')
    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file', default_value=default_config,
            description='USB 摄像头参数文件'),
        Node(
            package='v4l2_camera',
            executable='v4l2_camera_node',
            name='camera',
            namespace='/adas/sensors/camera',
            output='screen',
            parameters=[config],
        ),
    ])
