#!/usr/bin/env python3
import numpy as np 
from geometry_msgs.msg import Point
from rclpy.node import Node 
from std_msgs.msg import Header 

def order_points(corners : list[list[int]]) -> np.ndarray:
    """
    Returns given 2D coordinates in clockwise order, 
                top-left, top-right, bottom-right, bottom-left
    """
    corners = np.array(corners, dtype=np.int32)

    # sort by y axis 
    idx = np.argsort(corners[:,1])
    top = corners[idx[:2]]
    bottom = corners[idx[2:]]

    # sort left-right within each row  
    top = top[np.argsort(top[:,0])]
    bottom = bottom[np.argsort(bottom[:,0])]

    tl, tr = top 
    bl, br = bottom

    return np.array([tl, tr, br, bl])

def get_edge_pairs(rectangle_corners : list[list[int]])-> list[list[list[int]]]: 
    """
    Given corners in rectangular order (tl, tr, br, bl), 
    return edge pairs 

    Args:
        corners (list[list[int]]): Ordered rectangle 2D corners

    Returns:
        list[list[list[int]]]: 2D edge pairs of the rectangle 
    """
    # edge pairs around the rectangle 
    return [[rectangle_corners[i], 
             rectangle_corners[(i + 1) % len(rectangle_corners)]] 
            for i in range(len(rectangle_corners))]

