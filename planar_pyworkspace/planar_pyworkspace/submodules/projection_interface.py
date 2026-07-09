#!/usr/bin/env python3
# Point to Pixel interface for visualizations.
from abc import ABC, abstractmethod
import pyrealsense2 as rs2
from rclpy.node import Node 
from sensor_msgs.msg import CameraInfo


class PointToPixel(ABC):
    """
    Abstract class to define point to pixel projections
    """

    def __init__(self, node : Node, camera_info_topic : CameraInfo):
        """
        Initialize the class

        Args:
            node (Node): ROS2 Node
            msg (CameraInfo): camera intrinsics topic 
        """
        self._node = node
        self._intrinsics = None 
        self._rgb_camera_info = node.create_subscription(CameraInfo, 
                                                         camera_info_topic,
                                                         self.intrinsics_cb,
                                                         1)
        self._initialized = False 
        
    def is_initialized(self) -> bool: 
        return self._initialized

    def intrinsics_cb(self, msg : CameraInfo):
        """
        Camera intrinsics call back to be run once, 
        destroys the subscriber after intrinsics are set 

        Args:
            msg (CameraInfo): CameraInfo ROS2 topic for intrinsics
        """
        self._node.destroy_subscription(self._rgb_camera_info)
        self._rgb_camera_info = None 

        self._intrinsics = self._set_intrinsics(msg)

        if self._intrinsics is None: 
            raise RuntimeError(
                "_set_intrinsics() did not initialize _intrinsics"
            )
        self._initialized = True 
        

    @abstractmethod
    def _set_intrinsics(self, msg : CameraInfo):
        """
        Process the CameraInfo message to set camera intrinsics.
        Camera-specific.

        Args:
            msg (CameraInfo): CameraInfo ROS2 topic for intrinsics 
        """
        pass 

    @abstractmethod 
    def project_point_to_pixel(self, point : list[float]) -> list[int]: 
        """
        Camera specific routine to project the point to pixel. 

        Args:
            point (list[float]): 3D point wrt. camera coordinate space

        Returns:
            list[int]: 2D pixel coordinates wrt. Image coordinate space
        """
        pass 


class PointToPixelD400(PointToPixel):
    """
    Point to pixel class for D400 RealSense cameras
    """
    def __init__(self, node : Node, camera_info_topic : CameraInfo)-> None: 
        super().__init__(node, camera_info_topic)

    def _set_intrinsics(self, msg : CameraInfo):
        """
        Camera specific intrinsics
        """
        intrinsics = rs2.intrinsics()    
        intrinsics.width = msg.width
        intrinsics.height = msg.height
        intrinsics.ppx = msg.k[2]
        intrinsics.ppy = msg.k[5]
        intrinsics.fx = msg.k[0]
        intrinsics.fy = msg.k[4]
        if msg.distortion_model == 'plumb_bob':
            intrinsics.model = rs2.distortion.modified_brown_conrady
        intrinsics.coeffs = [i for i in msg.d]
        return intrinsics

    def project_point_to_pixel(self, point): 
        """
        Project 3D coordinate to 2D image coordinate

        Args:
            point (List[]): 3D coordinate (x,y,z) 

        Returns:
            List[]: 2D coordinate (x,y) in image coordinate system  
        """
        return rs2.rs2_project_point_to_pixel(self._intrinsics, 
                                              point)