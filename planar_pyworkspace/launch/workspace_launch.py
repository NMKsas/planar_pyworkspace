from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction

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
        TimerAction(
            period=1.0, # a second delay to make sure visualization node is ready 
            actions=[
                Node(
                    package='planar_pyworkspace',
                    executable='workspace_node',
                    name='workspace'
                )
            ]
        )
    ])