#!/usr/bin/env python3

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.impl import rcutils_logger
from tf2_ros import TransformBroadcaster, TransformException
from tf2_ros.buffer import Buffer 
from tf2_ros.transform_listener import TransformListener
from .utils.quaternion_utils import quaternion_from_axis_angle, \
                                    combine_quaternions

from .utils.file_utils import generate_csv
from .constants import (
    CAMERA_TF_FRAME,
    APRILTAG_CORNERS_CSV_FILE,
    DEFAULT_PATH,
)

ATTEMPTS = 15
DEBUG_ENABLED = True

# TODO: make corners configurable to user
CORNERS = {'bl':0.0, 'tl': -90.0, 'tr':-180.0, 'br':90.0}

class AprilTagPoseCollector(Node):
    """
    ROS 2 node for collecting AprilTag poses.
    """

    def __init__(self,
                 corner_frame_id : str,
                 camera_frame_id : str="st_cam_color_optical_frame", 
                 rot_about_z : float = 0.0,  
                 no_of_samples : int = 20, 
                 enable_debug_visualization : bool = False):
        """
        Args:
            marker_id (int): marker ID.
            no_of_samples (int, optional): Number of pose samples to collect.

        Args:
            child_frame (str): /tf frame ID of the marker as published by 
                               apriltag_node 
            rotation_about_z (float): correction about z in degrees. 
                                      + anticlockwise
                                      - clockwise 
        """
        super().__init__('apriltag_pose_collector_node')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self._camera_frame = camera_frame_id
        
        self._corner_frame = corner_frame_id
        self._rot_about_z = rot_about_z
        self._poses = []
        self._no_of_samples = no_of_samples

        self._mean = None
        self._variance = None

        self._enable_visualization = enable_debug_visualization

        if self._enable_visualization: 
            self.tf_broadcaster = TransformBroadcaster(self)

        self.timer = self.create_timer(0.05, self.collect_pose)

    
    def is_collected(self) -> bool: 
        """
        Returns true when all the corners are successfully collected
        """
        for _, v in self._corner_poses.items():
            if not v['is_collected']: 
                return False 
        return True 

    def collect_pose(self): 
        """
        Collect the pose of AprilTag corner coordinate. 
        If the coordinate frame direction needs adjusting wrt. plane, 
        rotation can be applied about z-axis.  

        """

        try: 
            # transformation from corner to camera 
            t = self.tf_buffer.lookup_transform(self._camera_frame, 
                                                self._corner_frame, rclpy.time.Time())
            # create a new sibling transformation 
            t.header.frame_id = self._camera_frame
            t.child_frame_id = self._corner_frame + '_rotated'

            x = t.transform.rotation.x
            y = t.transform.rotation.y
            z = t.transform.rotation.z
            w = t.transform.rotation.w
            
            # only rotation is corrected about plane z-axis 
            new_q = quaternion_from_axis_angle(np.array([0.0,0.0,1.0]), 
                                            np.radians(self._rot_about_z))
            new_q = combine_quaternions([x,y,z,w], new_q)

            t.transform.rotation.x = new_q[0]
            t.transform.rotation.y = new_q[1]
            t.transform.rotation.z = new_q[2]
            t.transform.rotation.w = new_q[3]


            self._poses.append([t.transform.translation.x,
                                t.transform.translation.y,
                                t.transform.translation.z, 
                                new_q[0],
                                new_q[1],
                                new_q[2],
                                new_q[3]])
            
            self.get_logger().info("Added pose sample no. " + str(len(self._poses)))

            if self._enable_visualization:
                self.tf_broadcaster.sendTransform(t)

        except TransformException as ex: 
            self.get_logger().info(f'Could not get transform from camera \
                                    frame to {self._corner_frame}: {ex}')

        if len(self._poses) == self._no_of_samples: 
            self._mean = np.mean(np.array(self._poses), axis=0)
            self._variance = np.var(np.array(self._poses), axis=0)

            self.timer.destroy()
            self.get_logger().info("All samples collected")


    def get_mean(self) -> np.array:
        """
        Returns:
            np.array: the mean of samples.
        """
        return self._mean

    def get_variance(self) -> np.array:
        """
        Returns:
            np.array: the variance of samples.
        """
        return self._variance

    def debug_print(self) -> None:
        """
        Print the collected pose list in a readable format for debugging.
        """
        str_print = (
            "\n=====================\n"
            + "AprilTag ID: " + str(self._corner_frame) + "\n"
            + "=====================\n"
        )
        if self._mean is None or self._variance is None:
            str_print += str(len(self._poses)) + ' samples collected.'
        else:
            str_print += (
                "Mean pose:\n"
                + f"\tx:{self._mean[0]}\n"
                + f"\ty:{self._mean[1]}\n"
                + f"\tz:{self._mean[2]}\n"
                + "Mean orientation:\n"
                + f"\tx:{self._mean[3]}\n"
                + f"\ty:{self._mean[4]}\n"
                + f"\tz:{self._mean[5]}\n"
                + f"\tw:{self._mean[6]}\n"
                + "---------------------\n"
                + "Variance pose\n"
                + f"\tx:{self._variance[0]}\n"
                + f"\ty:{self._variance[1]}\n"
                + f"\tz:{self._variance[2]}\n"
                + "Variance orientation:\n"
                + f"\tx:{self._variance[3]}\n"
                + f"\ty:{self._variance[4]}\n"
                + f"\tz:{self._variance[5]}\n"
                + f"\tw:{self._variance[6]}\n"
                + "====================="
            )

        self.get_logger().info(str_print)


def main(args=None):
    rclpy.init(args=args)
    logger = rcutils_logger.RcutilsLogger(name='apriltag_logger')
    logger.info('Collecting AprilTag pose samples')

    marker_poses = []
    collector = AprilTagPoseCollector('')

    for id, rot in CORNERS.items():
        apriltag_corner_collector = AprilTagPoseCollector(id,
                                                          rot_about_z=rot,
                                                          no_of_samples=20,
                                                          enable_debug_visualization=True)
        while rclpy.ok(): 
            
            rclpy.spin_once(apriltag_corner_collector)
            mean = apriltag_corner_collector.get_mean()
            variance = apriltag_corner_collector.get_variance()

            if mean is not None and variance is not None: 
                apriltag_corner_collector.debug_print()

                # add mean to marker poses list
                data = mean.tolist() 
                data.insert(0, CAMERA_TF_FRAME)
                data.insert(0, id)
                marker_poses.append(data)

                apriltag_corner_collector.destroy_node()
                break 
    

    logger.info('All AprilTag poses collected')

    generate_csv(
        APRILTAG_CORNERS_CSV_FILE,
        DEFAULT_PATH,
        [
            'Corner ID',
            'Parent frame ID',
            'position.x',
            'position.y',
            'position.z',
            'orientation.x',
            'orientation.y',
            'orientation.z',
            'orientation.w',
        ],
        marker_poses,
        False,
    )
    logger.info('CSV generated.')

    rclpy.shutdown()


if __name__ == '__main__':
    main()
