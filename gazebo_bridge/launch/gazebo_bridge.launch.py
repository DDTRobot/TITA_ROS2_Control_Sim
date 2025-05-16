#!/usr/bin/env python
import os
import launch
from launch import LaunchDescription
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

prefix="tita"

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
            default_value="gazebo_bridge",
            description="Define yaml file folder",
        )
    )
    # Get world file path
    world_file = os.path.join(
        FindPackageShare('gazebo_bridge').find('gazebo_bridge'),
        'worlds',
        'empty_world.world'
    )

    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('gazebo_ros'), 'launch'), '/gazebo.launch.py'
                ]),                    
                launch_arguments={
                    'world': world_file, 
                    'pause': 'false',
                    'verbose': 'false'
                }.items(),
            )
    
    spawn_entity = Node(package='gazebo_ros', 
                        executable='spawn_entity.py',
                        arguments=['-topic', f'{prefix}/robot_description',
                                   '-entity', f'{prefix}',
                                   '-x', '0.',
                                   '-y', '0.',
                                   '-z', '0.65'], 
                        output='screen')

    # save urdf
    robot_description_content_dir = PathJoinSubstitution(
        [FindPackageShare("tita_description"), "tita" , "xacro", LaunchConfiguration('urdf')]
    )
    xacro_executable = FindExecutable(name="xacro")

    robot_description_content = Command(
        [
            PathJoinSubstitution([xacro_executable]),
            " ",
            robot_description_content_dir,
            " ",
            "ctrl_mode:=",
            LaunchConfiguration('ctrl_mode'),
            " ",
            "sim_env:=",
            "gazebo",
            " ",
            "yaml_path:=",
            LaunchConfiguration('yaml_path'),
        ]
    )

    robot_description = {"robot_description": robot_description_content}

    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[
            robot_description,
            {'use_sim_time': True},
            {"publish_frequency":15.0},
            {"frame_prefix": prefix+"/"},
        ],
        namespace=prefix, 
    )

    nodes = [
        robot_state_pub_node,
        gazebo,
        spawn_entity,
    ]

    return LaunchDescription(declared_arguments + nodes)
