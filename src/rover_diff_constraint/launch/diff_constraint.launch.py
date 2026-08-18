import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    cfg = os.path.join(
        get_package_share_directory('rover_diff_constraint'),
        'config', 'diff_constraint.yaml')
    return LaunchDescription([
        Node(
            package='rover_diff_constraint',
            executable='diff_constraint_node',
            name='diff_constraint',
            parameters=[cfg],
            output='screen',
        ),
    ])
