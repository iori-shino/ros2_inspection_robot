"""Launch turtlesim and all inspection nodes with one shared config file."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    OpaqueFunction,
    RegisterEventHandler,
    TimerAction,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _launch_nodes(context):
    params_file = Path(
        LaunchConfiguration("params_file").perform(context)
    ).expanduser().resolve()
    if not params_file.is_file():
        raise RuntimeError(f"Parameter file does not exist: {params_file}")

    logger = Node(
        package="inspection_robot",
        executable="inspection_logger",
        name="inspection_logger",
        parameters=[str(params_file)],
        output="screen",
    )
    detector = Node(
        package="inspection_robot",
        executable="inspection_detector",
        name="inspection_detector",
        parameters=[str(params_file)],
        output="screen",
    )
    turtlesim = Node(
        package="turtlesim",
        executable="turtlesim_node",
        name="turtlesim",
        output="screen",
    )
    controller = Node(
        package="inspection_robot",
        executable="patrol_controller",
        name="patrol_controller",
        parameters=[str(params_file)],
        output="screen",
    )

    actions = []
    # Register before starting processes, including those that may fail early.
    for node in (logger, detector, turtlesim, controller):
        actions.append(
            RegisterEventHandler(
                OnProcessExit(
                    target_action=node,
                    on_exit=[EmitEvent(event=Shutdown(
                        reason="An inspection node exited."
                    ))],
                )
            )
        )
    actions.extend([
        logger,
        detector,
        turtlesim,
        # This is a startup allowance, not a ROS discovery/readiness check.
        TimerAction(period=2.0, actions=[controller]),
    ])
    return actions


def generate_launch_description():
    default_config = (
        Path(get_package_share_directory("inspection_robot"))
        / "config" / "inspection.yaml"
    )
    return LaunchDescription([
        DeclareLaunchArgument(
            "params_file",
            default_value=str(default_config),
            description="YAML configuration for the inspection nodes.",
        ),
        OpaqueFunction(function=_launch_nodes),
    ])
