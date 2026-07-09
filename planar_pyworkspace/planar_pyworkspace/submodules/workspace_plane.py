#!/usr/bin/env python3
# Class for defining and storing 3D workspace plane.

import numpy as np

LEFT_LOW = 0
LEFT_UP = 1
RIGHT_UP = 2
RIGHT_LOW = 3

class WorkspacePlane:
    """
    Workspace plane class to store a plane normal and plane limits. 
    """
    epsilon = 1e-8
    def __init__(self, corners : list[list[float]]):
        """
        3D workspace plane
        Args:
            corners: Corners (x,y,z) of the workspace plane as a List[]
        """

        self.corners = np.array(corners)
        self.workspace_limits = self._define_limits()
        self.normal = self._compute_plane_normal()

    def _compute_plane_normal(self) -> np.ndarray:
        """
        Calculate the norm of the plane, using three points
        on the plane.
        """

        p1 = self.corners[LEFT_UP]
        p2 = self.corners[RIGHT_UP]  
        p3 = self.corners[RIGHT_LOW] 

        normal_v = np.cross(p1 - p2, p3 - p2)

        # Normalize the normal vector
        normal = normal_v / (np.linalg.norm(normal_v) + self.epsilon)

        # Calculate the constant term for the plane equation
        const = -np.dot(normal, p3)

        # The plane equation in normal form
        return np.append(normal, const)

    def _define_limits(self) -> list[list[float]]:
        """
        Define the workspace limits, based on the corner 3D coordinates
        """
        min_x = np.min(self.corners, axis=0)[0] 
        min_y = np.min(self.corners, axis=0)[1] 
        min_z = np.min(self.corners, axis=0)[2]

        max_x = np.max(self.corners, axis=0)[0] 
        max_y = np.max(self.corners, axis=0)[1] 
        max_z = np.max(self.corners, axis=0)[2]
        return [[min_x, max_x], [min_y, max_y], [min_z, max_z]]

    def get_plane_normal(self) -> np.ndarray:
        """
        Returns: Plane function in a normal form
        """
        return self.normal

    def get_limits(self) -> list[list[float]]:
        """
        Returns: Workspace limits in 3D coordinates, 
        """
        return self.workspace_limits