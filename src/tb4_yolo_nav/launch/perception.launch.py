from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("display", default_value="true"),
            DeclareLaunchArgument("confidence", default_value="0.35"),
            Node(
                package="tb4_yolo_nav",
                executable="yolo_detector",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": True,
                        "display": ParameterValue(
                            LaunchConfiguration("display"), value_type=bool
                        ),
                        "confidence": ParameterValue(
                            LaunchConfiguration("confidence"), value_type=float
                        ),
                    }
                ],
            ),
        ]
    )
