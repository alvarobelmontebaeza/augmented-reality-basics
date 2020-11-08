#!/usr/bin/env python3

import os
import rospy
import numpy as np
import cv2
from image_geometry import PinholeCameraModel
from geometry_msgs.msg import Point

class Augmenter():
    '''
    Class that creates a wrapper between the duckietown simulator and the ROS
    interface of a fake robot simulated in the host. The node serves as interface
    for the following parts:
        - Subscribes to the wheels command topic and transforms it into actions in simulation
        - Publishes the rendered observations in the simulator to the camera image topic
    '''

    def __init__(self, camera_info, extrinsics):
        # Create camera model from camera calibration info
        self.cam_info = camera_info
        self.cam_model = PinholeCameraModel()
        self.cam_model.fromCameraInfo(self.cam_info)

        # Compute inverse of Projection Matrix
        self.homography_inv = np.linalg.inv(np.array(extrinsics['homography']).reshape((3,3)))
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
        Implementation based on: https://github.com/duckietown/dt-core/blob/952ebf205623a2a8317fcb9b922717bd4ea43c98/packages/image_processing/include/image_processing/rectification.py
        Args:
            raw_image: A CV image to be rectified
            interpolation: Type of interpolation. For more accuracy, use another cv2 provided constant
        Return:
            Undistorted image
        '''
        image_rectified = np.empty_like(raw_image)
        processed_image = cv2.remap(raw_image,self.mapx,self.mapy,interpolation,image_rectified)

        return processed_image
    
    def ground2pixel(self, ground_point_raw):
        '''
        Projects a point in the ground plane to a point in the image plane
        Implementation based on https://github.com/duckietown/dt-core/blob/952ebf205623a2a8317fcb9b922717bd4ea43c98/packages/image_processing/include/image_processing/ground_projection_geometry.py#L38
        Args:
            ground_point: numpy.array describing a 3D Point in ground coordinates to be transformed
        
        Returns: 
            tuple of pixel coordinates of the point in the image in normalized values (from 0 to 1)
        '''
        ground_point = Point()
        ground_point.x = ground_point_raw[0]
        ground_point.y = ground_point_raw[1]
        ground_point.z = ground_point_raw[2]

        if ground_point.z != 0:
            msg = 'This method assumes that the point is a ground point (z=0). '
            msg += 'However, the point is (%s,%s,%s)' % (ground_point.x, ground_point.y, ground_point.z)
            raise ValueError(msg)

        # Normalize ground point
        ground_point_norm = np.array([ground_point.x, ground_point.y, 1.0])
        # Transform the point to pixel coords
        image_point = np.dot(self.homography_inv, ground_point_norm)
        image_point = image_point / image_point[2]

        return image_point
    

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
    
    def render_segments(self, in_points, in_segments, image):
        # Extract the points and correct if necessary
        point_names = in_points.keys()
        corrected_points = dict.fromkeys(in_points.keys())
        i=0
        for in_point in in_points:
            in_point = np.array(in_point[1])
            if in_point[0] == 'axle':
                point = self.ground2pixel(in_point)
            elif in_point[0] == 'image01':
                point = in_point
            # Convert to absolute image coordinates
            pixel = [point[0] * self.cam_model.width, point[1] * self.cam_model.height]
            corrected_points[point_names[i]] = pixel
            i = i+1


        # Draw each described segment in the provided image
        for segment in in_segments:
            point1 = corrected_points[segment['points'][0]]
            point2 = corrected_points[segment['points'][1]]
            color = segment['color']

            rendered_image = self.draw_segment(image,point1,point2,color)


        return rendered_image