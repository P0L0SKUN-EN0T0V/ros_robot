from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    bridge_launch = (
        get_package_share_directory('foxglove_bridge')
        + '/launch/foxglove_bridge_launch.xml'
    )
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
        # foxglove_bridge — родной мост Foxglove Studio. ws://0.0.0.0:8765,
        # бинарный протокол, авто-схемы сообщений, корректный publishing
        # PoseStamped (rosbridge ROS2 ломает headers, foxglove_bridge — нет).
        IncludeLaunchDescription(
            AnyLaunchDescriptionSource(bridge_launch),
        ),
    ])
