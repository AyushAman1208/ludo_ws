"""Run the dice publisher."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("dice_seed", default_value="-1"),
        Node(
            package="ludo_game", executable="dice_publisher", name="dice_publisher",
            output="screen",
            parameters=[{
                "dice_seed": ParameterValue(LaunchConfiguration("dice_seed"), value_type=int),
            }],
        ),
    ])
