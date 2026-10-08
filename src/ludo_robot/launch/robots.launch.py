from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

COLORS = ("red", "green", "yellow", "blue")


def generate_launch_description():
    speed = LaunchConfiguration("speed")
    nodes = [
        Node(
            package="ludo_robot", executable="robot_controller",
            name=f"{c}_controller", output="screen",
            parameters=[{
                "color": c,
                "time_scale": ParameterValue(speed, value_type=float),
            }],
        )
        for c in COLORS
    ]
    return LaunchDescription([
        DeclareLaunchArgument("speed", default_value="1.0",
                              description="simulation speed; must equal ludo_sim realtime_factor"),
        *nodes,
    ])
