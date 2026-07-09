#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from planar_pyworkspace_interfaces.srv import PublishWorkplaneTf
from .utils.quaternion_utils import get_corner_quaternions
from .utils.ros_msg_utils import pose_to_tf
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster 

class WorkplaneBroadcaster(Node): 
    """
    Broadcaster node to publish static transformations between the 
    parent camera frame and the workspace corners upon service request  
    """

    def __init__(self, parent_frame : str="st_cam_color_optical_frame"): 
        """
        Args:
            parent_frame (str, optional): Parent frame for the published 
                                          transformations. Defaults to 
                                          "st_cam_color_optical_frame".
        """
        super().__init__('workplane_broadcaster')
        self._tf_broadcaster = TransformBroadcaster(self)
        self._vis_service = self.create_service(PublishWorkplaneTf,
                                                'publish_workplane_tf',
                                                self.update_tf)
        self._timer = None 
        self._parent_frame = parent_frame
        self._tl_tf_msg = TransformStamped()
        self._tr_tf_msg = TransformStamped()
        self._br_tf_msg = TransformStamped()
        self._bl_tf_msg = TransformStamped() 

        self.get_logger().info("Workplane broadcaster started")

    def update_tf(self, request : PublishWorkplaneTf.Request, 
                        response : PublishWorkplaneTf.Response) \
                        -> PublishWorkplaneTf.Response:
        """
        ROS2 service callback to enable workplane transformation broadcaster

        Returns:
            PublishWorkplaneTf.Response: Success true, when broadcaster is on. 
        """

        n = request.plane_normal
        tl = request.tl
        tr = request.tr
        br = request.br
        bl = request.bl

        tl_q, tr_q, br_q, bl_q = get_corner_quaternions(n[:3], [tl, tr, br, bl])

        self._tl_tf_msg = pose_to_tf(self, list(tl)+tl_q,
                                         self._parent_frame, 
                                         'workplane_top_left')
        
        self._tr_tf_msg = pose_to_tf(self, list(tr)+tr_q,
                                     self._parent_frame, 
                                     'workplane_top_right')
        
        self._br_tf_msg = pose_to_tf(self, list(br)+br_q,
                                     self._parent_frame, 
                                     'workplane_bottom_right')
        
        self._bl_tf_msg = pose_to_tf(self, list(bl)+bl_q,
                                     self._parent_frame, 
                                     'workplane_bottom_left')
        
        self._timer = self.create_timer(0.1, self.broadcast_timer_callback)

        self.get_logger().info("Workplane data received, starting tf broadcast")

        response.success = True 
        return response

    def broadcast_timer_callback(self)-> None: 
        """
        Broadcaster timer callback. Broadcasts transformations 
        from camera to corners of the workplane. 
        """
        stamp = self.get_clock().now().to_msg()

        self._tl_tf_msg.header.stamp = stamp
        self._tr_tf_msg.header.stamp = stamp
        self._br_tf_msg.header.stamp = stamp
        self._bl_tf_msg.header.stamp = stamp

        self._tf_broadcaster.sendTransform(self._tl_tf_msg)
        self._tf_broadcaster.sendTransform(self._tr_tf_msg)
        self._tf_broadcaster.sendTransform(self._bl_tf_msg)
        self._tf_broadcaster.sendTransform(self._br_tf_msg)

def main(): 
    rclpy.init()
    node = WorkplaneBroadcaster()

    try: 
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()


if __name__=="__main__":
    main() 