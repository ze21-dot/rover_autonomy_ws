#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_name = "rover_description"

    use_sim_time = LaunchConfiguration("use_sim_time")
    xacro_path = LaunchConfiguration("xacro_path")
    use_jsp_gui = LaunchConfiguration("use_jsp_gui")

    default_xacro = PathJoinSubstitution([
        FindPackageShare(pkg_name),
        "urdf",
        "karasimsekURDF.urdf.xacro",
    ])

    robot_description = ParameterValue(
        Command(["xacro", " ", xacro_path]),
        value_type=str
    )

    return LaunchDescription([
        # -----------------------------
        # Launch args
        # -----------------------------
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Use /clock (Isaac Sim) if true"
        ),
        DeclareLaunchArgument(
            "xacro_path",
            default_value=default_xacro,
            description="Absolute path to robot xacro file"
        ),
        DeclareLaunchArgument(
            "use_jsp_gui",
            default_value="true",
            description="Enable joint_state_publisher_gui (ONLY when Isaac is not publishing /joint_states)"
        ),

        # -----------------------------
        # robot_state_publisher
        # Publishes /tf_static (and /tf if you publish joints)
        # -----------------------------
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="screen",
            parameters=[{
                "use_sim_time": use_sim_time,
                "robot_description": robot_description,
            }],
        ),

        # -----------------------------
        # joint_state_publisher_gui
        # ONLY for visualization when you don't have real /joint_states
        # In Isaac Sim integration, keep this OFF.
        # -----------------------------
        Node(
            package="joint_state_publisher_gui",
            executable="joint_state_publisher_gui",
            output="screen",
            parameters=[{"use_sim_time": use_sim_time}],
            condition=IfCondition(use_jsp_gui),
        ),

        # -----------------------------
        # RViz
        # -----------------------------
        Node(
            package="rviz2",
            executable="rviz2",
            output="screen",
            parameters=[{"use_sim_time": use_sim_time}],
            arguments=["-d", PathJoinSubstitution([
                FindPackageShare(pkg_name),
                "rviz",
                "rover.rviz",
            ])],
        ),
    ])
