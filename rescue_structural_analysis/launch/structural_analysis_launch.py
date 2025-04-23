from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rescue_structural_analysis',
            executable='structural_analysis_node',
            name='structural_analysis_node',
            output='screen'
        )
    ])
