#!/usr/bin/env python3
# Static visualization layers. 

import numpy as np
import cv2
from abc import ABC, abstractmethod
from planar_pyworkspace.utils.point_utils import order_points, get_edge_pairs

class VisualizationLayer2D(ABC):
    """
    Abstract class for 2D visualization layers
    """
    @abstractmethod
    def __init__(self) -> None:
        print("Layer initialized")

    @abstractmethod
    def __call__(self, image : np.ndarray) -> np.ndarray:
        """
        Call function to apply modifications on the image 

        Args:
            image (np.ndarray): input image 

        Returns:
            np.ndarray: image with the layer applied
        """
        pass 

class SurfaceFillLayer(VisualizationLayer2D):
    """
    Visualization layer to paint 2D surface
    """
    def __init__(self, corners_2D : list[list[int]])-> None:
        """
        Args:
            width (int): image width
            height (int): image heigth
            corners_2D (list[list[int]]): List of corners
        """

        tl, tr, br, bl = order_points(corners_2D)
        self._corner_pairs = get_edge_pairs([tl,tr,br,bl])

    def __call__(self, image : np.ndarray) -> np.ndarray: 
        """
        Args:
            image (np.ndarray): image to draw on 

        Returns:
            np.ndarray: modified image
        """

        pts = np.array([p[0] for p in self._corner_pairs], dtype=np.int32)
        pts = pts.reshape((-1, 1, 2))

        overlay = image.copy()

        # fill on overlay
        cv2.fillPoly(overlay, [pts], (0, 0, 255))
        alpha = 0.5 
        cv2.addWeighted(overlay, alpha, image, 1 - alpha, 0, image)
        
        return image

class SurfaceBorderLayer(VisualizationLayer2D): 
    """
    Visualization layer for 2D surfaces
    """
    def __init__(self, corners_2D : list[list[int]])-> None:
        """
        Args:
            corners_2D (list[list[int]]): list of 2D corners
        """
        super().__init__()

        tl, tr, br, bl = order_points(corners_2D)
        self._corner_pairs = get_edge_pairs([tl,tr,br,bl])

    def __call__(self, image : np.ndarray) -> np.ndarray: 
        """
        Draw a surface on 2D image  
        """
        for p0, p1 in self._corner_pairs:
            cv2.line(image, tuple(p0), tuple(p1), (0, 0, 255), 2)

        return image
