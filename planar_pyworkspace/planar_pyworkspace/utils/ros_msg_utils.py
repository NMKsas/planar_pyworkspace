#!/usr/bin/env python3
from rclpy.node import Node 
from geometry_msgs.msg import Point, TransformStamped
from std_msgs.msg import Header 

def point_to_point_msg(p : list[float]) -> Point:
    """
    Convert a coordinate point list (x,y,z) into 
    ROS2 Point message 
    Args:
        p (list[float]): a coordinate point 

    Returns:
        Point: ROS2 Point msg 
    """

    point = Point() 
    point.x = p[0]
    point.y = p[1]
    point.z = p[2]
    return point 

def generate_header_msg(node : Node, frame : str) -> Header: 
    """
    Generate a ROS2 header

    Returns:
        Header: std_msgs/Header 
    """

    header = Header() 
    header.stamp = node.get_clock().now().to_msg()
    header.frame_id = frame
    return header 

def pose_to_tf(node : Node, pose : list[float], 
               parent_frame : str, child_frame : str) -> TransformStamped:
    """
    Bridge Pose to Transform, given a parent frame name and a name for the new 
    static frame. 

    Args:
        pose (List[x,y,z,x,y,z,w]): Pose and orientation as a list 
        parent_frame (str): The name of the parent frame 
        child_frame (str): The name of the created static frame 
    """
    t = TransformStamped()
    t.header.frame_id = parent_frame
    t.header.stamp = node.get_clock().now().to_msg()
    t.child_frame_id = child_frame
    t.transform.translation.x = pose[0]
    t.transform.translation.y = pose[1]
    t.transform.translation.z = pose[2]

    t.transform.rotation.x = pose[3]
    t.transform.rotation.y = pose[4]
    t.transform.rotation.z = pose[5]
    t.transform.rotation.w = pose[6] 
    
    return  t
