from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rescue_structural_analysis',
            executable='structural_analysis_node',
            name='structural_analysis_node',
            output='screen'    
        ),
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
        Node(
            package='rescue_communicator',
            executable='communicator_node',
            name='communicator_node',
            output='screen'
        ),
        Node(
            package='rescue_core_system',
            executable='core_system_node',
            name='core_system_node',
            output='screen'
        ),
        Node(
            package='rescue_actuator',
            executable='actuator_node',
            name='actuator_node',
            output='screen'
        ),
    ])
