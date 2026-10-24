from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os


def generate_launch_description():
    default_config = os.path.join(
        get_package_share_directory("jetson_rgbd_udp_sender"),
        "config",
        "sender.yaml",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "config_file",
                default_value=default_config,
                description="Sender parameter YAML file",
            ),
            DeclareLaunchArgument(
                "receiver_ip",
                default_value="192.168.1.107",
                description="Computer IP on the Mesh network",
            ),
            Node(
                package="jetson_rgbd_udp_sender",
                executable="rgbd_udp_sender",
                name="rgbd_udp_sender",
                output="screen",
                emulate_tty=True,
                parameters=[
                    LaunchConfiguration("config_file"),
                    {"receiver_ip": LaunchConfiguration("receiver_ip")},
                ],
            ),
        ]
    )
