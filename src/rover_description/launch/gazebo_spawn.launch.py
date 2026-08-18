#!/usr/bin/env python3
"""Spawn Karasimsek into Gazebo Harmonic.

  ros2 launch rover_description gazebo_spawn.launch.py                     # empty world
  ros2 launch rover_description gazebo_spawn.launch.py world:=marsyard     # Mars-yard testbed
  ... diff_joints:=revolute                                                # live rocker suspension
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            OpaqueFunction, SetEnvironmentVariable)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def setup(context, *args, **kwargs):
    pkg = "rover_description"
    share = get_package_share_directory(pkg)
    world = LaunchConfiguration("world").perform(context)
    diff_joints = LaunchConfiguration("diff_joints").perform(context)

    if world in ("empty", "empty.sdf"):
        world_name, gz_world = "empty", "empty.sdf"
    else:
        world_name = world.replace(".sdf", "")
        gz_world = os.path.join(share, "worlds", world_name + ".sdf")

    robot_description = ParameterValue(
        Command(["xacro ", os.path.join(share, "urdf", "karasimsekURDF.urdf.xacro"),
                 " diff_joints:=", diff_joints]),
        value_type=str)

    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory("ros_gz_sim"), "launch", "gz_sim.launch.py")),
            launch_arguments={"gz_args": f"-r {gz_world}"}.items()),
        Node(package="robot_state_publisher", executable="robot_state_publisher",
             output="screen",
             parameters=[{"use_sim_time": True, "robot_description": robot_description}]),
        Node(package="ros_gz_sim", executable="create", output="screen",
             arguments=["-topic", "robot_description", "-name", "karasimsek", "-z", "0.05"]),
        Node(package="ros_gz_bridge", executable="parameter_bridge", output="screen",
             arguments=[
                 "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
                 f"/world/{world_name}/model/karasimsek/joint_state@sensor_msgs/msg/JointState[gz.msgs.Model"],
             remappings=[(f"/world/{world_name}/model/karasimsek/joint_state", "/joint_states")],
             parameters=[{"use_sim_time": True}]),
    ]


def generate_launch_description():
    share_parent = os.path.dirname(get_package_share_directory("rover_description"))
    return LaunchDescription([
        SetEnvironmentVariable(
            name="GZ_SIM_RESOURCE_PATH",
            value=os.environ.get("GZ_SIM_RESOURCE_PATH", "") + os.pathsep + share_parent),
        DeclareLaunchArgument("world", default_value="empty",
                              description="empty | marsyard (from rover_description/worlds)"),
        DeclareLaunchArgument("diff_joints", default_value="fixed",
                              description="fixed | revolute (live rocker suspension)"),
        OpaqueFunction(function=setup),
    ])
