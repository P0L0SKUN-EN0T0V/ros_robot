from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='global_planner',
            executable='fake_sim_node.py',
            name='fake_sim_node',
            output='screen',
        ),
        Node(
            package='global_planner',
            executable='global_planner_node',
            name='global_planner_node',
            output='screen',
        ),
    ])
