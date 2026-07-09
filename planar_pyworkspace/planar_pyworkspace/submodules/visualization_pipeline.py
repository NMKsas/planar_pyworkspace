#!/usr/bin/env python3
# Wrapper class for visualization pipeline.

import numpy as np 
from .static_layers import VisualizationLayer2D
from .dynamic_layers import DynamicRosVisualizationLayer2D

class VisualizationPipeline2D():
    """
    Pipeline for 2D visualization layers 
    """
    def __init__(self) -> None:
        self._pipeline = []
    
    def add_layer(self, layer : 
                  VisualizationLayer2D | DynamicRosVisualizationLayer2D) \
                  -> None:
        """
        Adds layers to visualization pipeline

        Args:
            layer (VisualizationLayer2D | DynamicRosVisualizationLayer2D)
        """
        self._pipeline.append(layer)

    def remove_layer(self) -> None:
        """
        Remove layers from the visualization pipeline
        """
        self._pipeline.pop()

    def push(self, image : np.ndarray) -> np.ndarray: 
        """
        Pass the image through layer stack 

        Args:
            image (np.ndarray): input image

        Returns:
            np.ndarray: image passed through pipeline
        """
        for l in self._pipeline: 
            image = l(image)
        return image
