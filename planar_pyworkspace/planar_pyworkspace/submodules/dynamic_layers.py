#!/usr/bin/env python3
# Dynamic visualization layers.

from rclpy.node import Node 
import numpy as np
import time 
from abc import ABC, abstractmethod
import cv2

from geometry_msgs.msg import PointStamped
from .projection_interface import PointToPixel
from planar_pyworkspace_interfaces.srv import GestureVisConfig, \
                                              WorkplaneVisConfig, \
                                              KeypointArrowVisConfig, \
                                              DisableVisualization
from pose_detection_interfaces.msg import Pose2D
from planar_pyworkspace.utils.point_utils import order_points, get_edge_pairs


class DynamicRosVisualizationLayer2D(ABC):
    """
    Wrapper abstract class for visualizing ROS2 topic-based dynamic layers 
    """
    def __init__(self, node : Node) -> None: 
        """
        Args:
            node (Node): ROS2 node 
        """
        self._node = node 
        self._processor = self._noop 

    def __call__(self, image : np.ndarray) -> np.ndarray: 
        """
        Args:
            image (np.ndarray): input image

        Returns:
            np.ndarray: image with layer applied on it 
        """
        return self._processor(image)

    def _noop(self, image : np.ndarray) -> np.ndarray:
        """
        Set as the self._processor when the layer is disabled. 
        Image is passed through layer without changes. 

        Args:
            image (np.ndarray): input image

        Returns:
            np.ndarray: input image as-is 
        """
        return image
    
    @abstractmethod
    def _visualize(self, image : np.ndarray) -> np.ndarray: 
        """
        Set as the self._processor when the layer is enabled. 
        Applies layer visualizations to the image. 

        Args:
            image (np.ndarray): input image 

        Returns:
            np.ndarray: image with dynamic layer applied on it
        """
        pass  

    @abstractmethod
    def disable_visualization(self, request : DisableVisualization.Request,
                                    response : DisableVisualization.Response) \
                                    -> DisableVisualization.Response:
        """
        Disables dynamic visualizations. 
        Besides setting self._processor to self._noop, can be used to destroy 
        e.g., unused ROS2 topic subscriptions.
        """
        pass

    @abstractmethod
    def enable_visualization(self, request, response): 
        """
        Enables visualizaton.
        Besides setting self._processor to self._visualize, can be used to 
        initialize e.g., ROS2 topic subscriptions. 

        Args:
            request (ROS2 Service request): Service call definition
            response (ROS2 Service response): Response definition
        """
        pass 


class WorkplaneVisualizationLayer(DynamicRosVisualizationLayer2D):
    """
    Visualization layer for workplane. Draws a surface of four points. 
    """
    def __init__(self, node : Node, 
                 point_to_pixel_interface : PointToPixel) -> None:
        """
        Args:
            node (Node): ROS2 node
            point_to_pixel_interface (PointToPixel): projection interface
        """
        super().__init__(node)
        self._ptp_interface = point_to_pixel_interface
        self._workplane_service = node.create_service(WorkplaneVisConfig, 
                                                      'enable_workplane_layer',
                                                      self.enable_visualization)
        self._corner_pairs_2D = [] 
        self._h_norm = 0.0  # visualizing vector from left to right 
        self._v_norm = 0.0  # visualizing vector from bottom to top


    def _visualize(self, image : np.ndarray) -> np.ndarray:
        """
        Draw a workplane borders on 2D image.

        Args:
            image (np.ndarray): input image

        Returns:
            np.ndarray: image with layer applied on it 
        """
        for p0, p1 in self._corner_pairs_2D:
            cv2.line(image, tuple(p0), tuple(p1), (0, 0, 255), 2)

        return image

    def disable_visualization(self, request: DisableVisualization.Request,
                                    response: DisableVisualization.Response) \
                                    -> DisableVisualization.Response:
        """
        Disables visualizations. 

        Returns:
            DisableVisualization.Response: ROS2 service response 
        """
        self._processor = self._noop
        response.success = True 
        return response 

    def enable_visualization(self, request : WorkplaneVisConfig.Request, 
                                   response : WorkplaneVisConfig.Response) \
                                   -> WorkplaneVisConfig.Response: 
        """
        Enables visualizations. Projects given coordinates to 2D workplane, 
        and defines visualizing norms.

        Returns:
            WorkplaneVisConfig.Response: ROS2 service response
        """
        
        pts2D = []
        
        # project 3D coordinates into 2D image coordinates 
        pts2D.append(self._ptp_interface.project_point_to_pixel([request.p1.x, 
                                                                 request.p1.y, 
                                                                 request.p1.z]))
        pts2D.append(self._ptp_interface.project_point_to_pixel([request.p2.x, 
                                                                 request.p2.y, 
                                                                 request.p2.z]))
        pts2D.append(self._ptp_interface.project_point_to_pixel([request.p3.x, 
                                                                 request.p3.y, 
                                                                 request.p3.z]))
        pts2D.append(self._ptp_interface.project_point_to_pixel([request.p4.x, 
                                                                 request.p4.y, 
                                                                 request.p4.z]))
        # order the points 
        tl, tr, br, bl = order_points(pts2D)
        self._corner_pairs_2D = get_edge_pairs([tl,tr,br,bl])
        
        # normal as the mean of two options
        self._h_norm = 0.5*(np.linalg.norm(tr - tl) + np.linalg.norm(br - bl))
        self._v_norm = 0.5*(np.linalg.norm(bl - tl) + np.linalg.norm(br - tr))
        
        self._processor = self._visualize
        response.success = True 
        return response


class KeypointArrowVisualizationLayer(DynamicRosVisualizationLayer2D): 
    """
    Visualization layer for drawing arrows between selected body keypoints 
    """
    def __init__(self, node : Node, keypoint_topic : str) -> None:
        """
        Args:
            node (Node): ROS2 node
            keypoint_topic (str): topic for subscribing the keypoints
        """
        
        super().__init__(node)

        self._node = node
        self._vis_service = node.create_service(KeypointArrowVisConfig,
                                                'enable_arrow_pair_layer',
                                                self.enable_visualization)
        self._remove_service = node.create_service(DisableVisualization,
                                                   'disable_arrow_pair_layer',
                                                   self.disable_visualization)
        self._kp_topic = keypoint_topic
        self._keypoint_sub = None 

        # keypoint IDs for arrow head and tail 
        self._tail_kp_id = None 
        self._head_kp_id = None 
        self._tail_kp = None 
        self._head_kp = None 

        # required confidence [0,1]: initially set to 1.0
        self._confidence_threshold = 1.0 
        self._timestamp = 0.0
        self._display_time = 0.1 # seconds 
        
    def _visualize(self, image : np.ndarray) -> np.ndarray: 
        """
        Draws an arrow from an arrow tail keypoint to an arrow head keypoint. 

        Args:
            image (np.ndarray): input image

        Returns:
            np.ndarray: image with layer applied on it 
        """

        # Make sure the keypoints are recent enough 
        if time.monotonic() - self._timestamp > self._display_time: 
            return image 
        
        cv2.arrowedLine(image, (self._tail_kp.x, self._tail_kp.y),
                               (self._head_kp.x, self._head_kp.y),
                               [0, 255, 0], 2)
        return image 
    
    def keypoint_cb(self, msg : Pose2D) -> None:
        """
        ROS2 topic callback for keypoint detections.

        Args:
            msg (pose_detection_interfaces.msg.Pose2D): pose topic for keypoints
        """

        kp1 = msg.keypoint_list[self._tail_kp_id]
        kp2 = msg.keypoint_list[self._head_kp_id]

        # require confidence to be high enough 
        if kp1.conf >= self._confidence_threshold and \
           kp2.conf >= self._confidence_threshold: 
            
            self._tail_kp = kp1
            self._head_kp = kp2
            self._processor = self._visualize

        # record the timestamp
        self._timestamp = time.monotonic() 
 
    
    def disable_visualization(self, request : DisableVisualization.Request, 
                                    response: DisableVisualization.Response) \
                                    -> DisableVisualization.Response: 
        """
        Disables visualizations. 

        Returns:
            DisableVisualization.Response: ROS2 service response. 
        """

        # if ROS subscription exists, disable it
        if self._keypoint_sub is not None: 
            self._node.destroy_subscription(self._keypoint_sub)
            self._keypoint_sub = None
        
        # set processor to no operations 
        self._processor = self._noop
        response.success = True 
        return response 
    
    def enable_visualization(self, request : KeypointArrowVisConfig.Request, 
                                   response : KeypointArrowVisConfig.Response) \
                                   ->  KeypointArrowVisConfig.Response: 
        """
        Enable visualizations. Creates subscription for keypoint detection. 

        Returns:
            KeypointArrowVisConfig.Response: ROS2 service response
        """

        # define keypoint IDs, confidence
        self._tail_kp_id = request.tail_keypoint_id
        self._head_kp_id = request.head_keypoint_id 
        self._confidence_threshold = request.confidence_threshold

        # subscription for keypoints 
        self._keypoint_sub =  self._node.create_subscription(Pose2D, 
                                                             self._kp_topic,
                                                             self.keypoint_cb,
                                                             1)
        response.success = True 
        return response



class GestureVisualizationLayer(DynamicRosVisualizationLayer2D):
    """
    Visualization layer for pointing gestures. 
    """
    def __init__(self, node : Node, 
                 point_to_pixel_interface : PointToPixel,
                 workplane_layer : WorkplaneVisualizationLayer,
                 gesture_topic : str="/gesture_pointer/right_pointer") -> None:
        """
        Args:
            node (Node): ROS2 node 
            point_to_pixel_interface (PointToPixel): projection interface
            workplane_layer (WorkplaneVisualizationLayer): workplane reference 
            gesture_topic (str, optional): Gesturing topic to subscribe. 
                                Defaults to "/gesture_pointer/right_pointer".
        """
        super().__init__(node)
        self._vis_service = node.create_service(GestureVisConfig, 
                                                'enable_gesturing_layer', 
                                                self.enable_visualization)
        self._gesture_topic = gesture_topic
        self._topic_sub = None 
        self._workplane_layer = workplane_layer
        self._ellipse_delta_x = 0
        self._ellipse_delta_y = 0 
        self._x = 0 
        self._y = 0 
        self._timestamp = 0.0
        self._display_time = 0.1 # seconds
        self._ptp_interface = point_to_pixel_interface
    
    def _visualize(self, image : np.ndarray): 
        """
        Visualize the gestured point using a scaled ellipse and red dot. 

        Args:
            image (np.ndarray): input image

        Returns:
            np.ndarray: image with layer applied on it 
        """
        
        # Make sure the messages are recent enough 
        if time.monotonic() - self._timestamp > self._display_time: 
            return image 
        
        # red laser like pointer
        cv2.circle(image, (int(self._x), int(self._y)), 2, 
                    [255, 0, 0], 2)
        # scale ellipse, publish_visualized_image using plane dimensions
        cv2.ellipse(image, (int(self._x), int(self._y)),
                    (self._ellipse_delta_x, self._ellipse_delta_y),
                    angle=0, startAngle=0, endAngle=360, color=[0, 255, 0], 
                    thickness=1)
        
        return image
    
    def update_msg(self, msg : PointStamped) -> None:
        """
        Given gestured point, project the point into 2D image coordinates 
        and record timestamp. 

        Args:
            msg (PointStamped): (x,y,z) coordinate for the localized point
        """
        self._x,self._y = self._ptp_interface.project_point_to_pixel(
                                        [msg.point.x, msg.point.y, msg.point.z])
        self._timestamp = time.monotonic()
    
    def enable_visualization(self, request, response): 
        """ 
        Set the visualization values, initialize gesture subscription
        """
        # TODO: radius to match actual values (cm)
        self._ellipse_delta_x = int(self._workplane_layer._h_norm * request.radius)
        self._ellipse_delta_y = int(self._workplane_layer._v_norm * request.radius)

        # topic subscription 
        self._topic_sub = self._node.create_subscription(PointStamped, 
                                                         self._gesture_topic,
                                                         self.update_msg,
                                                         10)
        self._processor = self._visualize
        response.success = True 
        return response 

    def disable_visualization(self, request: DisableVisualization.Request, 
                                    response: DisableVisualization.Response) \
                                    -> DisableVisualization.Response:
        """
        Disables visualizations. 

        Returns:
            DisableVisualization.Response: ROS2 service response
        """

        # if ROS subscription exists, disable it
        if self._topic_sub is not None: 
            self._node.destroy_subscription(self._topic_sub)
            self._topic_sub = None

        # set processor to no operations 
        self._processor = self._noop
        response.success = True 
        return response 


