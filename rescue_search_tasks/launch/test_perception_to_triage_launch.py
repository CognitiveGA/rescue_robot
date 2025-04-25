from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rescue_perception',
            executable='perception_node',
            name='perception_node',
            output='screen'
        ),
        Node(
            package='rescue_search_tasks',
            executable='search_and_rescue_node',
            name='search_and_rescue_node',
            output='screen'
        ),
    ])
