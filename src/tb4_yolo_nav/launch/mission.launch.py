import os
from pathlib import Path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    root = Path(os.environ.get("TB4_WORKSPACE", "."))
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "route_file",
                default_value=str(root / "src/tb4_yolo_nav/config/goals.yaml"),
            ),
            DeclareLaunchArgument("initialize_pose", default_value="false"),
            Node(
                package="tb4_yolo_nav",
                executable="waypoint_mission",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": True,
                        "route_file": LaunchConfiguration("route_file"),
                        "initialize_pose": ParameterValue(
                            LaunchConfiguration("initialize_pose"), value_type=bool
                        ),
                    }
                ],
            ),
        ]
    )
