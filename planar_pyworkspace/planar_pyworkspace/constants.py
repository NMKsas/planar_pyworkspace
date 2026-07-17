#!/usr/bin/env python3

# Where all the files are stored 
DEFAULT_PATH = "/up/ros2env/src/planar_pyworkspace/planar_pyworkspace/data/"

# ArUco data collection constants 
CAMERA_TF_FRAME = 'st_cam_color_optical_frame'
CAMERA_RGB_TOPIC = '/camera/st_cam/color/image_raw'
CAMERA_INFO_TOPIC = '/camera/st_cam/color/camera_info'

APRILTAG_CORNERS_CSV_FILE = 'apriltag_corners_mean.csv'
ARUCO_CORNERS_CSV_FILE = 'aruco_corners_mean.csv'
VISUAL_CORNERS_CSV_FILE = 'visual_corners.csv'
DEFAULT_CSV = APRILTAG_CORNERS_CSV_FILE

# AprilTag data collection constants
APRILTAG_FAMILY = '36h11'
CORNER_MARKER_SIZE = 0.08 # m
MARKERS = [100,101,102,103]