#!/usr/bin/env python

import os
import pathlib
import launch
import xacro
from launch import LaunchDescription
from launch.substitutions.path_join_substitution import PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_launcher import WebotsLauncher
from webots_ros2_driver.webots_controller import WebotsController
from webots_ros2_driver.wait_for_controller_connection import (
    WaitForControllerConnection,
)

from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from webots_ros2_driver.urdf_spawner import URDFSpawner, get_webots_driver_node
from webots_ros2_driver.utils import controller_url_prefix


def generate_launch_description():
    webots_dir = get_package_share_directory('webots_bridge')
    control_dir = get_package_share_directory('tita_control')
    description_dir = get_package_share_directory('tita_description')
    
    webots = WebotsLauncher(
        world=PathJoinSubstitution([webots_dir, 'worlds', 'tita_arm.wbt']),
        ros2_supervisor=True
    )

    tita_description_path = os.path.join(description_dir, "urdf", "tita_control_with_arm.urdf")
    tita_ros2_control_params = os.path.join(control_dir, "config", "tita_ros2_controllers_with_arm.yaml")
    
    tita_driver = WebotsController(
        robot_name="tita",
        parameters=[{
            'robot_description': tita_description_path, 
            'use_sim_time': True,
            'set_robot_state_publisher': False,
            },
            tita_ros2_control_params,     
        ],
        respawn=True
    )
    
    # tita_driver = Node(
    #     package='webots_ros2_driver',
    #     executable='driver',
    #     output='screen',  # debugging
    #     additional_env={'WEBOTS_CONTROLLER_URL': controller_url_prefix() + 'tita'},
    #     parameters=[{
    #         'robot_description': tita_description_path, 
    #         'use_sim_time': True,
    #         'set_robot_state_publisher': False,
    #         },
    #         tita_ros2_control_params,     
    #     ],
    #     arguments=["tita_driver"],

    #     respawn=True
    # )
    
    # ROS2 control spawners 
    controller_manager_timeout = ["--controller-manager-timeout", "500"]
    controller_manager_prefix = "python.exe" if os.name == "nt" else ""

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        output="screen",
        prefix=controller_manager_prefix,
        arguments=["joint_state_broadcaster", "-c", "controller_manager"]
        + controller_manager_timeout,
    )
    
    imu_sensor_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        output="screen",
        prefix=controller_manager_prefix,
        arguments=["imu_sensor_broadcaster", "-c", "controller_manager"]
        + controller_manager_timeout,
    )
    
    tita_controller = Node(
            package='controller_manager',
            executable='spawner',
            output='screen',
            arguments=["tita_control", "-c", "controller_manager"]
            + controller_manager_timeout,
        )
    
    tita_spawners = [
        joint_state_broadcaster_spawner,
        # imu_sensor_broadcaster_spawner,
        # tita_controller,
    ]

    # # Wait for the simulation to be ready to start RViz, the navigation and spawners
    waiting_nodes = WaitForControllerConnection(
        target_driver=tita_driver, 
        nodes_to_start=tita_spawners
    )
    
    webots_event_handler = launch.actions.RegisterEventHandler(
        event_handler=launch.event_handlers.OnProcessExit(
            target_action=webots,
            on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
        )
    )
    
    return LaunchDescription(
        [
            webots,
            # webots._supervisor,

            tita_driver,
            waiting_nodes,
            
            webots_event_handler,
        ]
    )