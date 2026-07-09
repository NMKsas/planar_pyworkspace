#!/usr/bin/env python3
# Utils for handling quaternion algebra 

import numpy as np

def combine_quaternions(q1 : list[float], q2 : list[float]) -> list[float]:
    """
    Combines two quaternions

    Args:
        q1 (List[float]): First quaternion [x, y, z, w]
        q2 (List[float]): Second quaternion [x, y, z, w]

    Returns:
        List[float]: Combined quaternion
    """
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2

    x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
    y = w1 * y2 + y1 * w2 + z1 * x2 - x1 * z2
    z = w1 * z2 + z1 * w2 + x1 * y2 - y1 * x2
    w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2

    return [x, y, z, w]

def normalize(v : np.array) -> np.array: 
    """
    Normalize the given vector. Small epsilon added to avoid zero division.

    Args:
        v (np.array): vector

    Returns:
        np.array: unit vector 
    """
    return v / (np.linalg.norm(v)+ 1e-8)

def quaternion_from_axis_angle(r_axis : np.array, r_angle : np.array) -> list: 
    """
    Quaternion for rotating an angle about a rotation axis 

    Args:
        r_axis (np.array): Normalized axis for rotation 
        r_angle (np.array): Angle to be rotated in radians 
    Returns:
        List: quaternion for the orientation
    """
    w = np.cos(r_angle / 2)
    x,y,z = r_axis * np.sin(r_angle/2)
    return [x,y,z,w]


def get_workplane_orientation(n : np.array, p : list) -> list: 
    """
    Get quaternion for the given normal and plane points, i.e., 
    rotation that aligns with the plane. Currently based on orientation
    sequence: left low, left up, right up, right low. 

    Args:
        n (np.darray): plane normal as a list [a,b,c]
        p (List[]): coordinates for plane points 
    Returns:
        List[]: quaternion as list [x,y,z,w]
    """

    # z-axis as unit vector 
    z_axis = np.array([0,0,1])

    # rotation axis for aligning z-axis with normal 
    r_axis = np.cross(z_axis,n)
    r_axis = normalize(r_axis)

    # rotation angle 
    dot =  np.dot(z_axis, n)
    r_angle = np.arccos(dot)

    # quaternion for the alignment of xy-planes 
    q1 = quaternion_from_axis_angle(r_axis, r_angle)

    # for clarity, define components
    [x,y,z,w] = q1
    
    # y unit vector of the 1st rotation 
    y_unit = np.array([2*(x*y-z*w),
                       1-2*(x**2+z**2),
                       2*(y*z+x*w)])       
    y_unit = normalize(y_unit)

    # y unit vector on wanted coordinate system
    d2 = np.array(p[1]) - np.array(p[0])
    d2 = normalize(d2)

    # rotate about normal to align y-axes
    dot2 = np.dot(y_unit,d2)
    r2_angle = np.arccos(dot2) 

    # make sure the direction of the rotation is correct 
    cross2 = np.cross(y_unit, d2)
    if np.dot(cross2, n) < 0:
        r2_angle = -r2_angle

    # second quaternion  
    q2 = quaternion_from_axis_angle(n, r2_angle)
    
    # combine for perfect alignment
    q2q1 = combine_quaternions(q2,q1)
    

    return q2q1 


def get_corner_quaternions(n : np.array, p : list) -> list: 
    """
    Get quaternion for the given normal and plane points, i.e., 
    rotation that aligns with the plane. Currently based on orientation
    sequence: top left, top right, bottom right, bottom left. 

    Args:
        n (np.darray): plane normal as a list [a,b,c]
        p (List[]): coordinates for plane points 
    Returns:
        List[]: quaternion as list [x,y,z,w]
    """

    # z-axis as unit vector 
    z_axis = np.array([0,0,1])

    # rotation axis for aligning z-axis with normal 
    r_axis = np.cross(z_axis,n)
    r_axis = normalize(r_axis)

    # rotation angle 
    dot =  np.dot(z_axis, n)
    r_angle = np.arccos(dot)

    # quaternion for the alignment of xy-planes 
    q1 = quaternion_from_axis_angle(r_axis, r_angle)

    # for clarity, define components
    [x,y,z,w] = q1
    
    # y unit vector of the 1st rotation 
    y_unit = np.array([2*(x*y-z*w),
                       1-2*(x**2+z**2),
                       2*(y*z+x*w)])       
    y_unit = normalize(y_unit)

    corner_quaternions = []

    # y unit vector on wanted coordinate system
    for i in range(4): 
        
        py_tmp = np.array(p[(i+1)%4]) - np.array(p[i])
        py_unit = normalize(py_tmp)

        # rotate about normal to align y-axes
        dot2 = np.dot(y_unit, py_unit)
        r2_angle = np.arccos(dot2) 

        # make sure the direction of the rotation is correct 
        cross2 = np.cross(y_unit, py_unit)
        if np.dot(cross2, n) < 0:
            r2_angle = -r2_angle

        # second quaternion  
        q2 = quaternion_from_axis_angle(n, r2_angle)
        corner_quaternions.append(combine_quaternions(q2,q1))
    
    return corner_quaternions