#!/usr/bin/env python3

import os
import rospy
import numpy as np
import cv2
from image_geometry import PinholeCameraModel

class Augmenter():
    '''
    Class that creates a wrapper between the duckietown simulator and the ROS
    interface of a fake robot simulated in the host. The node serves as interface
    for the following parts:
        - Subscribes to the wheels command topic and transforms it into actions in simulation
        - Publishes the rendered observations in the simulator to the camera image topic
    '''

    def __init__(self, camera_info):
        # Create camera model from camera calibration info
        self.cam_info = camera_info
        self.cam_model = PinholeCameraModel()
        self.cam_model.fromCameraInfo(self.cam_info)

        # Initiate rectify maps
        self._init_rectify_maps()
    
    def _init_rectify_maps(self):
        W = self.cam_model.width
        H = self.cam_model.height
        mapx = np.ndarray(shape=(H,W,1), dtype='float32')
        mapy = np.ndarray(shape=(H,W,1), dtype='float32')
        mapx, mapy = cv2.initUndistortRectifyMap(self.cam_model.K, self.cam_model.D,self.cam_model.R,self.cam_model.P,(W,H),cv2.CV_32FC1,mapx,mapy)

        self.mapx = mapx
        self.mapy = mapy       

    def process_image(self, raw_image, interpolation=cv2.INTER_NEAREST):
        '''
        Undistort a provided image using the calibrated camera info

        Args:
            raw_image: A CV image to be rectified
            interpolation: Type of interpolation. For more accuracy, use another cv2 provided constant
        Return:
            Undistorted image
        '''
        image_rectified = np.empty_like(raw_image)
        processed_image = cv2.remap(raw_image,self.mapx,self.mapy,interpolation,image_rectified)

        return processed_image
    
    def ground2pixel(self, ground_points):
        return image_points
    
    def render_segments(self, segments, image):
        return rendered_image
    
    def draw_segment(self, image, pt_x, pt_y, color):
        defined_colors = {
            'red': ['rgb', [1, 0, 0]],
            'green': ['rgb', [0, 1, 0]],
            'blue': ['rgb', [0, 0, 1]],
            'yellow': ['rgb', [1, 1, 0]],
            'magenta': ['rgb', [1, 0 , 1]],
            'cyan': ['rgb', [0, 1, 1]],
            'white': ['rgb', [1, 1, 1]],
            'black': ['rgb', [0, 0, 0]]}
        _color_type, [r, g, b] = defined_colors[color]
        cv2.line(image, (pt_x[0], pt_y[0]), (pt_x[1], pt_y[1]), (b * 255, g * 255, r * 255), 5)
        return image