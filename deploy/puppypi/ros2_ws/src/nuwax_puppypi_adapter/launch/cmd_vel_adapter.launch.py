from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument("input_cmd_vel", default_value="/cmd_vel"),
            DeclareLaunchArgument("output_velocity", default_value="/puppy_control/velocity"),
            DeclareLaunchArgument("max_x_cm_s", default_value="12.0"),
            DeclareLaunchArgument("max_yaw_rate", default_value="0.6"),
            Node(
                package="nuwax_puppypi_adapter",
                executable="cmd_vel_adapter",
                name="puppypi_cmd_vel_adapter",
                output="screen",
                parameters=[
                    {
                        "input_cmd_vel": LaunchConfiguration("input_cmd_vel"),
                        "output_velocity": LaunchConfiguration("output_velocity"),
                        "max_x_cm_s": LaunchConfiguration("max_x_cm_s"),
                        "max_yaw_rate": LaunchConfiguration("max_yaw_rate"),
                    }
                ],
            ),
        ]
    )
