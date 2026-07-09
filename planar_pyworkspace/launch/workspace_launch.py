from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='planar_pyworkspace',
            executable='broadcaster_node',
            name='tf_broadcaster'
        ),
        Node(
            package='planar_pyworkspace',
            executable='visualizer_node',
            name='visualizer'
        ),
        Node(
            package='planar_pyworkspace',
            executable='workspace_node',
            name='workspace'
        )
    ])