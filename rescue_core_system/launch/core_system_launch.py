from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rescue_core_system',
            executable='core_system_node',
            name='core_system_node',
            output='screen'
        )
    ])