from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    package_share = FindPackageShare("rmgo_description")
    default_model = PathJoinSubstitution([package_share, "urdf", "omni_infantry.urdf.xacro"])
    default_rviz = PathJoinSubstitution([package_share, "rviz", "display.rviz"])

    model = LaunchConfiguration("model")
    rviz_config = LaunchConfiguration("rvizconfig")
    gui = LaunchConfiguration("gui")
    hardware_plugin = LaunchConfiguration("hardware_plugin")

    robot_description = ParameterValue(
        Command(
            [
                FindExecutable(name="xacro"),
                " ",
                model,
                " hardware_plugin:=",
                hardware_plugin,
            ]
        ),
        value_type=str,
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("gui", default_value="true"),
            DeclareLaunchArgument("model", default_value=default_model),
            DeclareLaunchArgument("rvizconfig", default_value=default_rviz),
            DeclareLaunchArgument(
                "hardware_plugin", default_value="gz_ros2_control/GazeboSimSystem"),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                parameters=[
                    {
                        "robot_description": robot_description,
                        "use_robot_description_topic": False,
                    }
                ],
                output="screen",
            ),
            Node(
                package="joint_state_publisher_gui",
                executable="joint_state_publisher_gui",
                condition=IfCondition(gui),
                parameters=[{"robot_description": robot_description}],
                output="screen",
            ),
            Node(
                package="joint_state_publisher",
                executable="joint_state_publisher",
                condition=UnlessCondition(gui),
                parameters=[{"robot_description": robot_description}],
                output="screen",
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                arguments=["-d", rviz_config],
                output="screen",
            ),
        ]
    )
