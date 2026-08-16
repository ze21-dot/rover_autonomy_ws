#!/usr/bin/env python3
"""GZ-2: spawn the rover into Gazebo Harmonic (empty world) with clock + joint_states bridged.

Usage:
  ros2 launch rover_description gazebo_spawn.launch.py
  ros2 launch rover_description gazebo_spawn.launch.py diff_joints:=revolute   # enable rocker suspension
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg = "rover_description"
    diff_joints = LaunchConfiguration("diff_joints")

    # Let gz-sim resolve model://rover_description/... URIs without manual exports
    share_parent = os.path.dirname(get_package_share_directory(pkg))
    gz_resource_path = SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value=os.environ.get("GZ_SIM_RESOURCE_PATH", "") + os.pathsep + share_parent)

    xacro_path = PathJoinSubstitution(
        [FindPackageShare(pkg), "urdf", "karasimsekURDF.urdf.xacro"])

    robot_description = ParameterValue(
        Command(["xacro ", xacro_path, " diff_joints:=", diff_joints]),
        value_type=str)

    return LaunchDescription([
        gz_resource_path,

        DeclareLaunchArgument("diff_joints", default_value="fixed",
                              description="fixed = frozen suspension (stable first spawn); revolute = rocker active"),

        # Gazebo Harmonic, empty world, autostart (-r)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution(
                [FindPackageShare("ros_gz_sim"), "launch", "gz_sim.launch.py"])),
            launch_arguments={"gz_args": "-r empty.sdf"}.items(),
        ),

        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="screen",
            parameters=[{"use_sim_time": True,
                         "robot_description": robot_description}],
        ),

        # Spawn from /robot_description, slightly above ground so it settles
        Node(
            package="ros_gz_sim",
            executable="create",
            output="screen",
            arguments=["-topic", "robot_description",
                       "-name", "karasimsek",
                       "-z", "0.03"],
        ),

        # Bridges: sim clock (CNV-3) + joint states from the gz JointStatePublisher plugin
        Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            output="screen",
            arguments=[
                "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
                "/world/empty/model/karasimsek/joint_state@sensor_msgs/msg/JointState[gz.msgs.Model",
            ],
            remappings=[("/world/empty/model/karasimsek/joint_state", "/joint_states")],
            parameters=[{"use_sim_time": True}],
        ),
    ])
