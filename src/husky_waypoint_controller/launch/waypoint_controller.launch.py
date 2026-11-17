"""Launch the Husky waypoint controller with its parameter file."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    """Start ``waypoint_controller`` with ``params_file`` and ``use_sim_time``."""
    params_file = LaunchConfiguration("params_file")
    use_sim_time = LaunchConfiguration("use_sim_time")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("husky_waypoint_controller"), "config", "params.yaml"]
                ),
                description="Controller parameters (all limits live here)",
            ),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="true",
                description="Use Gazebo /clock (true in simulation, false on hardware)",
            ),
            Node(
                package="husky_waypoint_controller",
                executable="waypoint_controller",
                name="waypoint_controller",
                output="screen",
                parameters=[params_file, {"use_sim_time": ParameterValue(use_sim_time, value_type=bool)}],
            ),
        ]
    )
