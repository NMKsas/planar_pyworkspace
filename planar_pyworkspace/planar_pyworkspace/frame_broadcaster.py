#!/usr/bin/env python3
import numpy as np
import rclpy
from rclpy.node import Node
from planar_pyworkspace_interfaces.srv import PublishWorkplaneTf, PublishPupilTf
from .utils.quaternion_utils import get_corner_quaternions, \
                                    quaternion_from_axis_angle
from .utils.ros_msg_utils import pose_to_tf
from geometry_msgs.msg import TransformStamped
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster

class WorkplaneTfBroadcaster(Node): 
    """
    Broadcaster node to publish static transformations between the 
    parent camera frame and the workspace corners upon service requests  
    """

    def __init__(self, parent_frame : str="st_cam_color_optical_frame"): 
        """
        Args:
            parent_frame (str, optional): Parent frame for the published 
                                          transformations. Defaults to 
                                          "st_cam_color_optical_frame".
        """
        super().__init__('workplane_broadcaster')
        self._tf_static_broadcaster = StaticTransformBroadcaster(self)

        self._tf_workplane_srv = self.create_service(PublishWorkplaneTf,
                                                     'publish_workplane_tf',
                                                     self.publish_workplane_tf)
        self._tf_pupil_srv = self.create_service(PublishPupilTf,
                                                 'publish_pupil_tf',
                                                 self.publish_pupil_origo)
        self._parent_frame = parent_frame
        self.get_logger().info("Workplane broadcaster started")


    def publish_workplane_tf(self, req : PublishWorkplaneTf.Request, 
                                   res : PublishWorkplaneTf.Response) \
                             -> PublishWorkplaneTf.Response:
        """
        ROS2 service callback to enable workplane transformation broadcaster

        Returns:
            PublishWorkplaneTf.Response: Success true, when broadcaster is on. 
        """

        n = req.plane_normal
        tl = req.tl
        tr = req.tr
        br = req.br
        bl = req.bl

        tl_q, tr_q, br_q, bl_q = get_corner_quaternions(n[:3], [tl, tr, br, bl])

        tl_tf_msg = pose_to_tf(self, list(tl)+tl_q,
                                         self._parent_frame, 
                                         'workplane_top_left')
        
        tr_tf_msg = pose_to_tf(self, list(tr)+tr_q,
                                     self._parent_frame, 
                                     'workplane_top_right')
        
        br_tf_msg = pose_to_tf(self, list(br)+br_q,
                                     self._parent_frame, 
                                     'workplane_bottom_right')
        
        bl_tf_msg = pose_to_tf(self, list(bl)+bl_q,
                                     self._parent_frame, 
                                     'workplane_bottom_left')
        

        self.get_logger().info("Workplane data received, starting tf broadcast")
        self._tf_static_broadcaster.sendTransform([tl_tf_msg, tr_tf_msg,
                                                   br_tf_msg, bl_tf_msg])
        res.success = True 
        return res

    def publish_pupil_origo(self, req : PublishPupilTf.Request, 
                                  res : PublishPupilTf.Response) \
                                     -> PublishPupilTf.Response: 
        """
        Publish the pupil origo frame 

        Returns:
            PublishPubilTf.Response: success when published
        """
        t = TransformStamped()

        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = req.parent_frame
        t.child_frame_id = 'pupil_origo'
        t.transform.translation.x = req.offset_mm / 1000
        t.transform.translation.y = req.offset_mm / 1000 
        t.transform.translation.z = 0.0

        # rotation is corrected about plane z-axis 
        q = quaternion_from_axis_angle(np.array([0.0,0.0,1.0]), 
                                       np.radians(req.deg_rot_about_z))

        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        self.get_logger().info("Starting Pupil origo /tf broadcast")
        self._tf_static_broadcaster.sendTransform(t)

        res.success = True 
        return res 


def main(): 
    rclpy.init()
    node = WorkplaneTfBroadcaster()

    try: 
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()


if __name__=="__main__":
    main() 