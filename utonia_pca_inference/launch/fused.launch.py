from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    receiver_config = PathJoinSubstitution(
        [FindPackageShare("rgbd_udp_receiver"), "config", "receiver.yaml"]
    )
    inference_config = PathJoinSubstitution(
        [FindPackageShare("utonia_pca_inference"), "config", "inference.yaml"]
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument("bind_address", default_value="0.0.0.0"),
            DeclareLaunchArgument("h265_port", default_value="5000"),
            DeclareLaunchArgument("zdepth_port", default_value="5001"),
            DeclareLaunchArgument("calibration_port", default_value="5002"),
            Node(
                package="utonia_pca_inference",
                executable="rgbd_utonia_node",
                output="screen",
                parameters=[
                    receiver_config,
                    inference_config,
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
