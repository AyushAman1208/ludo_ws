"""Only the game manager (use when the simulator and robot controllers are already running)."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("auto_start", default_value="true"),
        DeclareLaunchArgument("dice_seed", default_value="-1"),
        DeclareLaunchArgument("first_player", default_value="red"),
        Node(
            package="ludo_game", executable="game_manager", name="game_manager",
            output="screen",
            parameters=[{
                "auto_start": ParameterValue(LaunchConfiguration("auto_start"), value_type=bool),
                "dice_seed": ParameterValue(LaunchConfiguration("dice_seed"), value_type=int),
                "first_player": LaunchConfiguration("first_player"),
            }],
        ),
    ])
