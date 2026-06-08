from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    controller_manager = LaunchConfiguration("controller_manager")

    return LaunchDescription(
        [
            DeclareLaunchArgument("controller_manager", default_value="/controller_manager"),
            Node(
                package="controller_manager",
                executable="spawner",
                output="screen",
                arguments=[
                    "joint_state_broadcaster",
                    "omni_wheel_controller",
                    "chassis_power_controller",
                    "chassis_controller",
                    "teleop_remote_controller",
                    "gimbal_position_controller",
                    "--activate-as-group",
                    "--controller-manager",
                    controller_manager,
                ],
            ),
        ]
    )
