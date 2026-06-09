import os

import yaml
from launch import LaunchContext, LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction, SetEnvironmentVariable, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def load_controller_names(config_file: str) -> list[str]:
    with open(config_file, "r", encoding="utf-8") as file:
        config = yaml.safe_load(file) or {}

    controller_parameters = config.get("controller_manager", {}).get("ros__parameters", {})
    controller_names = [
        name
        for name, parameters in controller_parameters.items()
        if isinstance(parameters, dict) and "type" in parameters
    ]
    if not controller_names:
        raise RuntimeError(f"No controllers with a 'type' entry found in {config_file}")

    return controller_names


def launch_setup(context: LaunchContext, *args, **kwargs):
    del args, kwargs

    robot_config = LaunchConfiguration("robot").perform(context)
    if robot_config.startswith("auto."):
        is_automatic = True
        robot_name = robot_config[5:]
    else:
        is_automatic = False
        robot_name = robot_config

    model_config = LaunchConfiguration("model").perform(context)
    model_name = model_config if model_config else robot_name

    bringup_share = FindPackageShare("rmgo_bringup").perform(context)
    description_share = FindPackageShare("rmgo_description").perform(context)
    ros_gz_sim_share = FindPackageShare("ros_gz_sim").perform(context)

    config_file = os.path.join(bringup_share, "config", robot_name + ".yaml")
    model_file = os.path.join(description_share, "urdf", model_name + ".urdf.xacro")
    gazebo_resource_path = os.path.dirname(description_share)
    existing_gazebo_resource_path = os.environ.get("GZ_SIM_RESOURCE_PATH", "")
    controller_names = load_controller_names(config_file)

    robot_description = ParameterValue(
        Command(
            [
                FindExecutable(name="xacro"),
                " ",
                model_file,
                " controllers_file:=",
                config_file,
                " hardware_plugin:=",
                LaunchConfiguration("hardware_plugin"),
                " debug_visuals:=",
                LaunchConfiguration("debug_visuals"),
            ]
        ),
        value_type=str,
    )

    controller_manager = LaunchConfiguration("controller_manager")

    return [
        LogInfo(
            msg=(
                f"Starting RMGO on robot '{robot_config}'"
                f"{'(automatic)' if is_automatic else ''} -> {robot_name}.yaml, model {model_name}.urdf.xacro"
            )
        ),
        LogInfo(msg=f"Spawning controllers from {robot_name}.yaml: {', '.join(controller_names)}"),
        SetEnvironmentVariable(
            name="GZ_SIM_RESOURCE_PATH",
            value=(
                gazebo_resource_path
                if not existing_gazebo_resource_path
                else gazebo_resource_path + os.pathsep + existing_gazebo_resource_path
            ),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(ros_gz_sim_share, "launch", "gz_sim.launch.py")),
            launch_arguments={"gz_args": LaunchConfiguration("gz_args")}.items(),
        ),
        Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
            output="screen",
        ),
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
            package="ros_gz_sim",
            executable="create",
            parameters=[{"robot_description": robot_description}],
            arguments=[
                "-world",
                LaunchConfiguration("world"),
                "-param",
                "robot_description",
                "-name",
                robot_name,
                "-allow_renaming",
                "true",
                "-z",
                LaunchConfiguration("spawn_z"),
            ],
            output="screen",
        ),
        TimerAction(
            period=LaunchConfiguration("controller_spawn_delay"),
            actions=[
                Node(
                    package="controller_manager",
                    executable="spawner",
                    output="screen",
                    arguments=[
                        *controller_names,
                        "--activate-as-group",
                        "--controller-manager",
                        controller_manager,
                        "--controller-manager-timeout",
                        LaunchConfiguration("controller_manager_timeout"),
                    ],
                ),
            ],
        ),
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("robot", default_value="omni_infantry"),
            DeclareLaunchArgument("model", default_value=""),
            DeclareLaunchArgument("world", default_value="empty"),
            DeclareLaunchArgument("gz_args", default_value="-r -s empty.sdf"),
            DeclareLaunchArgument("spawn_z", default_value="0.2"),
            DeclareLaunchArgument("controller_manager", default_value="/controller_manager"),
            DeclareLaunchArgument("controller_spawn_delay", default_value="5.0"),
            DeclareLaunchArgument("controller_manager_timeout", default_value="60.0"),
            DeclareLaunchArgument("hardware_plugin", default_value="gz_ros2_control/GazeboSimSystem"),
            DeclareLaunchArgument("debug_visuals", default_value="false"),
            OpaqueFunction(function=launch_setup),
        ]
    )
