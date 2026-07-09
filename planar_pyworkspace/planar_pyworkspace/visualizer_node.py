#!/usr/bin/env python3
# Workplane visualizer 

from cv_bridge import CvBridge
import rclpy
from rclpy.node import Node 
from sensor_msgs.msg import Image as ROS_Image

from .submodules.visualization_pipeline import VisualizationPipeline2D
from .submodules.dynamic_layers import DynamicRosVisualizationLayer2D, \
                                       GestureVisualizationLayer, \
                                       WorkplaneVisualizationLayer, \
                                       KeypointArrowVisualizationLayer
from .submodules.projection_interface import PointToPixelD400

    
class WorkplaneVisualizer(): 
    """
    Visualization manager to control the 2D visualizations 
    """
    def __init__(self, node : Node,
                 ptp_interface : PointToPixelD400,
                 layers : list[DynamicRosVisualizationLayer2D],
                 rgb_topic : str="/camera/st_cam/color/image_raw"):
        """
        Args:
            node (Node): ROS2 node
            ptp_interface (PointToPixelD400): projection interface
            layers (list[DynamicRosVisualizationLayer2D]): list of layers 
            rgb_topic (str, optional): ROS2 topic for RGB stream.
        """

        # corresponding node class
        self._node = node 
        self._rgb_topic = rgb_topic
        self._bridge = CvBridge() 

        # unregister the subscriber
        self._ptp_interface = ptp_interface
        self._pipeline = self._init_visual_pipeline(layers)
        
        # start subscription and post-processing  
        self._rgb_sub = self._node.create_subscription(ROS_Image, 
                                                       self._rgb_topic, 
                                                       self.rgb_cb, 
                                                       1)
        self._rgb_pub = self._node.create_publisher(ROS_Image, 
                                                    "/visualizer/image_raw",
                                                    1)
        
        self._node.get_logger().info("Visualizer initialized")

    def _init_visual_pipeline(self, 
                              layers : list[DynamicRosVisualizationLayer2D]) \
                              -> VisualizationPipeline2D:
        """
        Iniatialize the pipeline 

        Args:
            layers (list[DynamicRosVisualizationLayer2D]): layers to be added 
                                                           to pipeline
        """

        self._pipeline = VisualizationPipeline2D()

        for l in layers: 
            self._pipeline.add_layer(l)

        return self._pipeline

    def rgb_cb(self, image_data : ROS_Image) -> None: 
        """
        Callback for the regular image. The images are saved in cv2 format. 
        """
        self._rgb_pub.publish(
            self._bridge.cv2_to_imgmsg(
                self._pipeline.push(
                    self._bridge.imgmsg_to_cv2(image_data, image_data.encoding)
                )
            , encoding="rgb8"))


def main(args=None):
    rclpy.init(args=args)

    node = rclpy.create_node("visualization_node")

    ptp_interface = PointToPixelD400(
        node,
        "/camera/st_cam/color/camera_info"
    )

    while not ptp_interface.is_initialized():
        rclpy.spin_once(node, timeout_sec=1.0)
        node.get_logger().info(
            f"Waiting for PointToPixel Interface to initialize" 
        )

    workplane_layer = WorkplaneVisualizationLayer(node, ptp_interface)
    layers = [
                workplane_layer, 
                GestureVisualizationLayer(node, ptp_interface, workplane_layer, 
                                          "gesture_pointer/left_pointer"),
                GestureVisualizationLayer(node, ptp_interface, workplane_layer, 
                                          "gesture_pointer/right_pointer"),
                KeypointArrowVisualizationLayer(node, 'pose_keypoints')
             ]

    vis_node = WorkplaneVisualizer(node, ptp_interface, layers)

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
