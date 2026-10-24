from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config_file = PathJoinSubstitution(
        [FindPackageShare("rgbd_udp_receiver"), "config", "receiver.yaml"]
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument("bind_address", default_value="0.0.0.0"),
            DeclareLaunchArgument("h265_port", default_value="5000"),
            DeclareLaunchArgument("zdepth_port", default_value="5001"),
            DeclareLaunchArgument("calibration_port", default_value="5002"),
            Node(
                package="rgbd_udp_receiver",
                executable="rgbd_udp_receiver",
                name="rgbd_udp_receiver",
                output="screen",
                parameters=[
                    config_file,
                    {
                        "bind_address": LaunchConfiguration("bind_address"),
                        "h265_port": ParameterValue(
                            LaunchConfiguration("h265_port"), value_type=int
                        ),
                        "zdepth_port": ParameterValue(
                            LaunchConfiguration("zdepth_port"), value_type=int
                        ),
                        "calibration_port": ParameterValue(
                            LaunchConfiguration("calibration_port"), value_type=int
                        ),
                    },
                ],
            ),
        ]
    )
