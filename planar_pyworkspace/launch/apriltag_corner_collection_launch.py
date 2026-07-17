from launch import LaunchDescription
from launch.actions import Shutdown
from launch_ros.actions import Node

from planar_pyworkspace.constants import MARKERS, CORNER_MARKER_SIZE, \
                                         CAMERA_RGB_TOPIC, CAMERA_INFO_TOPIC, \
                                         APRILTAG_FAMILY

def generate_launch_description():
    """
    Launch description for detecting multiple markers and collecting the 
    poses with apriltag pose collector node 

    Returns:
        LaunchDescription 
    """
    
    ld = LaunchDescription()

    apriltag_params = {
        'image_transport': 'raw',
        'family': APRILTAG_FAMILY,
        'size': CORNER_MARKER_SIZE,
        'profile': False,

        'max_hamming': 0,

        'detector': {
            'threads': 1,
            'decimate': 2.0,
            'blur': 0.0,
            'refine': True,
            'sharpening': 0.25,
            'debug': False,
        },

        'pose_estimation_method': 'pnp',

        'tag': {
            'ids': MARKERS,
            'sizes': [CORNER_MARKER_SIZE] * len(MARKERS),
            'frames' : ['bl', 'tl', 'tr', 'br']
        },
    }

    # apriltag node responsible for detections 
    ld.add_action(
        Node(
            package='apriltag_ros',
            executable='apriltag_node',
            name='apriltag',
            parameters=[apriltag_params],
            remappings=[
                ('image_rect', CAMERA_RGB_TOPIC),
                ('camera_info', CAMERA_INFO_TOPIC),
            ],
            output='screen',
        )
    )
    
    # collector node
    ld.add_action(
        Node(
            package='planar_pyworkspace',
            executable='apriltag_pose_collector',
            arguments=["--number_of_cycles", "1"],
            on_exit=Shutdown()
        )
    )

    return ld


