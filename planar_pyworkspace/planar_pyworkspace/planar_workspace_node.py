#!/usr/bin/env python3
# Workspace class for defining a 2D plane for gesturing. 
from rclpy.node import Node
import rclpy 

from .submodules.workspace_plane import WorkspacePlane
from .utils.ros_msg_utils import point_to_point_msg, generate_header_msg
from .utils.file_utils import read_corners
from .constants import DEFAULT_PATH, DEFAULT_CSV
from planar_pyworkspace_interfaces.srv import WorkplaneVisConfig, GetWorkplane, \
                                              PublishWorkplaneTf

class PlanarWorkspace(Node):
    """
    ROS2 node wrapper for a 3D workspace with a 2D workplane. 
    Defined using 3D coordinates in a given coordinate frame system.
    """

    def __init__(self, workspace_corners : list[list[float]],
                       coordinate_frame : str)-> None:
        """
        Args:
            workspace_corners (list[list[float]]): Pre-defined list of corner 
                                                   3D coordinates (x,y,z)
            coordinate_frame (str): ROS2 coordinate frame_id corresponding to the
                                    given coordinates 
        """
        super().__init__('workspace_node')

        self._workplane = WorkspacePlane(workspace_corners)
        self._workspace_frame = coordinate_frame
        self._request_visualization(workspace_corners)
        self._request_tf_broadcaster(workspace_corners)
        self._vis_service = self.create_service(GetWorkplane, 
                                                'get_workplane', 
                                                self.get_workplane_cb)


    def _request_visualization(self, workspace_corners : list[list[float]]) \
                               -> None: 
        """
        ROS2 Service request for enabling workspace visualization

        Raises:
            RuntimeError: Service call to 'enable_workplane_layer' fails 
        """

        self._vis_client = self.create_client(WorkplaneVisConfig,
                                              'enable_workplane_layer')
        if not self._vis_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().info("Visualization service 'enable_workplane_layer' not available")
            return 
        
        vis_request = WorkplaneVisConfig.Request()
        vis_request.p1 = point_to_point_msg(workspace_corners[0])
        vis_request.p2 = point_to_point_msg(workspace_corners[1])
        vis_request.p3 = point_to_point_msg(workspace_corners[2])
        vis_request.p4 = point_to_point_msg(workspace_corners[3])
        vis_request.header = generate_header_msg(self, self._workspace_frame)

        future = self._vis_client.call_async(vis_request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        
        if not future.done(): 
            raise RuntimeError("Service call 'enable_workplane_layer' failed")
        
        response = future.result()
        if response.success: 
            self.get_logger().info("Visualization for workplane enabled")
        else: 
            self.get_logger().info("Failed to initialize visualization. Disabled.")

    def _request_tf_broadcaster(self, workspace_corners : list[list[float]]) \
                                -> None: 
        """
        ROS2 request for enabling TF broadcaster for the workspace corners 
        """
        self._tf_client = self.create_client(PublishWorkplaneTf,
                                            'publish_workplane_tf')
        if not self._tf_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().info("Tf broadcaster 'publish_workplane_tf' not available")
            return 
        
        tf_request = PublishWorkplaneTf.Request()
        tf_request.header = generate_header_msg(self, self._workspace_frame)
        tf_request.plane_normal = self._workplane.get_plane_normal()
        tf_request.tl = workspace_corners[0]
        tf_request.tr = workspace_corners[1]
        tf_request.br = workspace_corners[2]
        tf_request.bl = workspace_corners[3]

        future = self._tf_client.call_async(tf_request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        
        if not future.done(): 
            self.get_logger().info("Service call 'publish_workplane_tf' failed. Broadcaster disabled.")
        
        response = future.result()
        if response.success: 
            self.get_logger().info("Tf broadcaster for workplane enabled")
        else: 
            self.get_logger().info("Failed to initialize tf broadcaster. Disabled.")    

    def get_workplane_cb(self, request : GetWorkplane.Request, 
                               response : GetWorkplane.Response) \
                               -> GetWorkplane.Response: 
        """
        ROS2 service for sending workplane definition

        Args:
            request (GetWorkplane.Request): ROS2 service request for workplane
            response (GetWorkplane.Response): ROS2 service response 

        Returns:
            GetWorkplane.Response: Definition of workplane 
        """

        self.get_logger().info("Request for workplane received")

        limits_x, limits_y, limits_z = self._workplane.get_limits()
        normal = self._workplane.get_plane_normal()
        response.limits_x = limits_x
        response.limits_y = limits_y
        response.limits_z = limits_z
        response.plane_normal = normal 
        response.header = generate_header_msg(self, self._workspace_frame)
        response.success = True 
        return response 

def main(args=None):
    rclpy.init(args=args)

    corners = read_corners(DEFAULT_CSV, DEFAULT_PATH)

    workspace_node = PlanarWorkspace(corners, "st_cam_color_frame")
    rclpy.spin(workspace_node)

    workspace_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
