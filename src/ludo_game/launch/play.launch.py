"""Start the simulator and dice publisher.

    ros2 launch ludo_game play.launch.py
    ros2 launch ludo_game play.launch.py viewer:=false speed:=3.0 dice_seed:=42
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def _include(pkg, name, args):
    path = os.path.join(get_package_share_directory(pkg), "launch", name)
    return IncludeLaunchDescription(PythonLaunchDescriptionSource(path), launch_arguments=args.items())


def generate_launch_description():
    viewer = LaunchConfiguration("viewer")
    cameras = LaunchConfiguration("render_cameras")
    speed = LaunchConfiguration("speed")
    return LaunchDescription([
        DeclareLaunchArgument("viewer", default_value="true"),
        DeclareLaunchArgument("render_cameras", default_value="false",
                              description="publish robot camera images (costs CPU)"),
        DeclareLaunchArgument("speed", default_value="1.0",
                              description="simulation speed relative to wall-clock time"),
        DeclareLaunchArgument("dice_seed", default_value="-1"),
        DeclareLaunchArgument("dice_rate", default_value="0.2",
                              description="dice messages per second"),
        _include("ludo_sim", "ludo_sim.launch.py",
                 {"viewer": viewer, "render_cameras": cameras, "realtime_factor": speed}),
        _include("ludo_game", "game.launch.py",
                 {"dice_seed": LaunchConfiguration("dice_seed"),
                  "dice_rate": LaunchConfiguration("dice_rate")}),
    ])
