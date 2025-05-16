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
from webots_ros2_driver.wait_for_controller_connection import (
    WaitForControllerConnection,
)
from webots_ros2_driver.urdf_spawner import URDFSpawner, get_webots_driver_node
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    declared_arguments = []
    # declared_arguments.append(
    #     DeclareLaunchArgument(
    #         "prefix",
    #         default_value='test_prefix/',
    #         description="Prefix of the joint names, useful for \
    #     multi-robot setup. If changed than also joint names in the controllers' configuration \
    #     have to be updated.",
    #     )
    # )
    # prefix = LaunchConfiguration("prefix")
    prefix="webots"
    webots = WebotsLauncher(
        world=PathJoinSubstitution(
            [FindPackageShare("titati_webots_bridge"), "worlds", "titati.wbt"]
        ),
        ros2_supervisor=True,
    )

    # save urdf
    robot_description_content_dir = PathJoinSubstitution(
        [FindPackageShare("titati_webots_bridge"), "urdf", "titati.urdf.xacro"]
    )
    xacro_executable = FindExecutable(name="xacro")
    urdf_output_path = '/tmp/titati_controller/titati_robot_description.urdf'
    urdf_output_dir = os.path.dirname(urdf_output_path)
    os.makedirs(urdf_output_dir, exist_ok=True)
    save_urdf_process = ExecuteProcess(
        cmd=[
            xacro_executable, robot_description_content_dir,
            '>', urdf_output_path
        ],
        shell=True
    )
    robot_description_content = Command(
        [
            PathJoinSubstitution([xacro_executable]),
            " ",
            robot_description_content_dir,
        ]
    )

    robot_description = {"robot_description": robot_description_content}
    robot_controllers = os.path.join(
        get_package_share_directory("titati_controller"),
        "config",
        "titati_ros2_controllers.yaml",
    )

    tita_driver = WebotsController(
        robot_name="titati_webots",
        parameters=[
            robot_description,
            robot_controllers,
            {"robot_description_path": urdf_output_path},
            {"use_sim_time": True},
            {"frame_prefix": prefix+"/"},
        ],
        respawn=True,
        namespace=prefix,
        # remappings=[
        #     ("/webots/tita1/cmd_vel", "/asssssss/asd_vel"),
        # ], 
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
    
    # ROS2 control spawners
    controller_manager_timeout = ["--controller-manager-timeout", "500"]
    controller_manager_prefix = "python.exe" if os.name == "nt" else ""

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        output="screen",
        prefix=controller_manager_prefix,
        arguments=["joint_state_broadcaster", "-c", prefix+"/controller_manager"]
        + controller_manager_timeout,
    )

    imu_sensor_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        output="screen",
        prefix=controller_manager_prefix,
        arguments=["imu_sensor_broadcaster", "-c", prefix+"/controller_manager"]
        + controller_manager_timeout,
    )

    tita_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        output="screen",
        prefix=controller_manager_prefix,
        arguments=["titati_controller", "-c", prefix+"/controller_manager"]
        + controller_manager_timeout,
    )

    tita_spawners = [
        joint_state_broadcaster_spawner,
        imu_sensor_broadcaster_spawner,
        tita_controller_spawner,
    ]

    # Wait for the simulation to be ready to start RViz, the navigation and spawners
    waiting_nodes = WaitForControllerConnection(
        target_driver=tita_driver, nodes_to_start=tita_spawners
    )

    webots_event_handler = launch.actions.RegisterEventHandler(
        event_handler=launch.event_handlers.OnProcessExit(
            target_action=webots,
            on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
        )
    )
    meshes_source_dir = PathJoinSubstitution(
        [FindPackageShare("titati_webots_bridge"), "meshes"]
    )
    meshes_destination_dir = "/tmp/titati_description/meshes"
    os.makedirs(os.path.dirname(meshes_destination_dir), exist_ok=True)
    copy_meshes_process = ExecuteProcess(
        cmd=["cp", "-r", meshes_source_dir, meshes_destination_dir],
        shell=True
    )
    
    nodes = [
        webots,
        webots._supervisor,
        save_urdf_process,
        copy_meshes_process,
        robot_state_pub_node,
        tita_driver,
        waiting_nodes,
        webots_event_handler,
    ]

    return LaunchDescription(declared_arguments + nodes)
