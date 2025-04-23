from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rescue_communicator',
            executable='communicator_node',
            name='communicator_node',
            output='screen'
        )
    ])
