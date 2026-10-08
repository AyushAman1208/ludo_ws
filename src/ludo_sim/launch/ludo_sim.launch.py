from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("viewer", default_value="true",
                              description="open the MuJoCo viewer window"),
        DeclareLaunchArgument("render_cameras", default_value="true",
                              description="publish robot camera images"),
        DeclareLaunchArgument("camera_rate", default_value="10.0"),
        DeclareLaunchArgument("realtime_factor", default_value="1.0",
                              description="simulation speed relative to wall clock"),
        Node(
            package="ludo_sim", executable="mujoco_node", name="mujoco_node",
            output="screen",
            parameters=[{
                "viewer": ParameterValue(LaunchConfiguration("viewer"), value_type=bool),
                "render_cameras": ParameterValue(LaunchConfiguration("render_cameras"), value_type=bool),
                "camera_rate": ParameterValue(LaunchConfiguration("camera_rate"), value_type=float),
                "realtime_factor": ParameterValue(LaunchConfiguration("realtime_factor"), value_type=float),
            }],
        ),
        Node(package="ludo_sim", executable="state_publisher", name="state_publisher",
             output="screen"),
    ])
