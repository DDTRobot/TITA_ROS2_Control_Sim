#!/usr/bin/env python
import os
import launch
from launch import LaunchDescription
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_launcher import WebotsLauncher
from webots_ros2_driver.webots_controller import WebotsController

from webots_ros2_driver.urdf_spawner import URDFSpawner, get_webots_driver_node
from launch.actions import DeclareLaunchArgument
from launch.actions import OpaqueFunction

prefix="tita"

def launch_setup(context, *args, **kwargs):
    yaml_path = LaunchConfiguration('yaml_path')
    urdf = LaunchConfiguration('urdf')
    ctrl_mode = LaunchConfiguration('ctrl_mode')
    webots = WebotsLauncher(
        world=PathJoinSubstitution(
            [FindPackageShare("webots_bridge"), "worlds", "tita.wbt"]
        ),
        ros2_supervisor=True,
    )
    webots_event_handler = launch.actions.RegisterEventHandler(
        event_handler=launch.event_handlers.OnProcessExit(
            target_action=webots,
            on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
        )
    )
    robot_description_content_dir = PathJoinSubstitution(
        [FindPackageShare("tita_description"), "tita" , "xacro", urdf]
    )
    xacro_executable = FindExecutable(name="xacro")

    robot_description_content = Command(
        [
            PathJoinSubstitution([xacro_executable]),
            " ",
            robot_description_content_dir,
            " ",
            "ctrl_mode:=",
            ctrl_mode,
            " ",
            "sim_env:=",
            "webots",
        ]
    )

    robot_description = {"robot_description": robot_description_content}

    yaml_path_string = yaml_path.perform(context)
    robot_controllers = os.path.join(
        get_package_share_directory(yaml_path_string),
        "config",
        "controllers.yaml",
    )

    tita_driver = WebotsController(
        robot_name="tita_webots",
        parameters=[
            robot_description,
            robot_controllers,
            {"use_sim_time": True},
        ],
        respawn=True,
        namespace=prefix,
    )

    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[
            robot_description,
            {"frame_prefix": prefix+"/"},
        ],
        namespace=prefix, 
    )

    return [
        webots,
        webots._supervisor,
        robot_state_pub_node,
        tita_driver,
        webots_event_handler,
    ]

def generate_launch_description():
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "ctrl_mode",
            default_value="wbc",
            choices=["wbc", "sdk", "mcu"],
            description="Enable sdk of joint effort input",
        )
    )    
    declared_arguments.append(
        DeclareLaunchArgument(
            "urdf",
            default_value="robot.xacro",
            description="Define urdf file in description folder",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "yaml_path",
            default_value="webots_bridge",
            description="Define yaml file folder",
        )
    )
    return LaunchDescription(
        declared_arguments  + 
        [OpaqueFunction(function=launch_setup)]
    )
