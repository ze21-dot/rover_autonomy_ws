#!/usr/bin/env python3
"""GZ-3: Gazebo spawn + double-Ackermann IK + command bridges.
Drive with:  ros2 run teleop_twist_keyboard teleop_twist_keyboard
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    spawn = IncludeLaunchDescription(PythonLaunchDescriptionSource(
        os.path.join(get_package_share_directory("rover_description"),
                     "launch", "gazebo_spawn.launch.py")))

    ik = Node(
        package="rover_kinematics",
        executable="ackermann_ik_node",
        output="screen",
        parameters=[
            os.path.join(get_package_share_directory("rover_kinematics"),
                         "config", "kinematics.yaml"),
            {"use_sim_time": True},
        ],
    )

    # ROS -> GZ command bridges (8 topics; ']' = ROS-to-GZ direction)
    cmd_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        output="screen",
        arguments=[f"/karasimsek/cmd/{t}@std_msgs/msg/Float64]gz.msgs.Double"
                   for t in ("fl_steer", "fr_steer", "rl_steer", "rr_steer",
                             "fl_wheel", "fr_wheel", "rl_wheel", "rr_wheel")],
        parameters=[{"use_sim_time": True}],
    )

    return LaunchDescription([spawn, ik, cmd_bridge])
