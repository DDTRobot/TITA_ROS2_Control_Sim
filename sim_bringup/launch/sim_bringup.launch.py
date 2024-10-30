#!/usr/bin/env python
import os
from launch import LaunchDescription, LaunchContext
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PythonExpression

prefix="tita"

def generate_launch_description():
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "sim_env",
            default_value="gazebo",
            description="Select simulation environment",
            choices=["webots", "gazebo"]
        )
    )

    webots_controller_manager_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('webots_bridge'),
            'launch',
            'webots_bridge.launch.py'
        )),
        launch_arguments={
            'urdf': "robot.xacro",
            'yaml_path': "sim_bringup",
        }.items(),
        condition=IfCondition(PythonExpression(["'", LaunchConfiguration('sim_env'), "' == 'webots'"]))
    )

    gazebo_controller_manager_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('gazebo_bridge'),
            'launch',
            'gazebo_bridge.launch.py'
        )),
        launch_arguments={
            'urdf': "robot.xacro",
            'yaml_path': "sim_bringup",
        }.items(),
        condition=IfCondition(PythonExpression(["'", LaunchConfiguration('sim_env'), "' == 'gazebo'"]))
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", prefix+"/controller_manager"],
    )
    
    imu_sensor_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["imu_sensor_broadcaster", "--controller-manager", prefix+"/controller_manager"],
    )

    
    effort_controller = Node(
        package='controller_manager',
        # output='screen',
        executable='spawner',
        arguments=["effort_controller", "--controller-manager", prefix+"/controller_manager"],
    )

    template_controller = Node(
        package='controller_manager',
        # output='screen',
        executable='spawner',
        arguments=["template_ros2_controller", "--controller-manager", prefix+"/controller_manager"],
    )

    return LaunchDescription(declared_arguments + [
        webots_controller_manager_launch,
        gazebo_controller_manager_launch,

        joint_state_broadcaster_spawner,
        imu_sensor_broadcaster_spawner,
        effort_controller,
        # template_controller,
 ])
