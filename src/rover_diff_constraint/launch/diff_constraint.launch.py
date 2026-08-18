import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    cfg = os.path.join(
        get_package_share_directory('rover_diff_constraint'),
        'config', 'diff_constraint.yaml')
    topics = [
        "/model/karasimsek/joint/leftjoint/cmd_force",
        "/model/karasimsek/joint/rightjoint/cmd_force",
    ]
    bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge', output='screen',
        arguments=[f"{t}@std_msgs/msg/Float64]gz.msgs.Double" for t in topics],
        parameters=[{'use_sim_time': True}])
    node = Node(
        package='rover_diff_constraint', executable='diff_constraint_node',
        name='diff_constraint', parameters=[cfg, {'use_sim_time': True}],
        output='screen')
    return LaunchDescription([bridge, node])
