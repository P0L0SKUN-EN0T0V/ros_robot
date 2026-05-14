from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    rosbridge_launch = (
        get_package_share_directory('rosbridge_server')
        + '/launch/rosbridge_websocket_launch.xml'
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
        # ROS ↔ WebSocket мост на ws://0.0.0.0:9090. Foxglove Studio
        # подключается к нему напрямую и сам отрисовывает occupancy
        # grid, lidar, path. Самописного web_viz больше нет.
        IncludeLaunchDescription(
            AnyLaunchDescriptionSource(rosbridge_launch),
        ),
    ])
