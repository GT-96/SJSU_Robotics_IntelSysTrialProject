from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='path_planner',
            executable='goal_and_start',
            name='goal_and_start_node',
            output='screen'
        ),
        Node(
            package='path_planner',
            executable='map',
            name='map_node',
            output='screen'
        ),
        Node(
            package='path_planner',
            executable='path',
            name='path_node',
            output='screen'
        ),
        Node(
            package='path_planner',
            executable='viz',
            name='viz_node',
            output='screen'
        )
    ])
