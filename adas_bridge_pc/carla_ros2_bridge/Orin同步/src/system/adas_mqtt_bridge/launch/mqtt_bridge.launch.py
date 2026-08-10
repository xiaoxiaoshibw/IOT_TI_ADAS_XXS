"""ADAS MQTT Bridge — ROS2 launch file"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('mqtt_broker', default_value='broker.emqx.io',
                              description='MQTT Broker 地址'),
        DeclareLaunchArgument('mqtt_port', default_value='1883',
                              description='MQTT Broker 端口'),
        DeclareLaunchArgument('topic_prefix', default_value='adas/v1',
                              description='MQTT 主题前缀'),

        Node(
            package='adas_mqtt_bridge',
            executable='mqtt_bridge_node.py',
            name='adas_mqtt_bridge',
            namespace='',
            output='screen',
            parameters=[{
                'mqtt_broker': LaunchConfiguration('mqtt_broker'),
                'mqtt_port': LaunchConfiguration('mqtt_port'),
                'topic_prefix': LaunchConfiguration('topic_prefix'),
            }],
            emulate_tty=True,
        ),
    ])
