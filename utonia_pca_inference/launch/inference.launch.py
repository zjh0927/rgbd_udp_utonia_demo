from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config_file = PathJoinSubstitution(
        [FindPackageShare("utonia_pca_inference"), "config", "inference.yaml"]
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument("input_topic", default_value="/rgbd/color_points"),
            DeclareLaunchArgument("output_topic", default_value="/utonia/pca_points"),
            Node(
                package="utonia_pca_inference",
                executable="utonia_pca_node",
                name="utonia_pca_inference",
                output="screen",
                parameters=[
                    config_file,
                    {
                        "input_topic": LaunchConfiguration("input_topic"),
                        "output_topic": LaunchConfiguration("output_topic"),
                    },
                ],
            ),
        ]
    )
