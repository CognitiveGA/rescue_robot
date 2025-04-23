from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rescue_actuator',
            executable='actuator_node',
            name='actuator_node',
            output='screen'
        )
    ])
