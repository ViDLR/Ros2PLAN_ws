from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='user_visualization_interface',
            executable='uservisualization_node',
            name='uservisualization_node',
            output='screen',
        )
    ])
