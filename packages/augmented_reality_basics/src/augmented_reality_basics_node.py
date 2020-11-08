#!/usr/bin/env python3

import os
import sys
import rospy
import numpy as np
import cv2
from cv_bridge import CvBridge
import rospkg
import yaml
from duckietown.dtros import DTROS, NodeType
from sensor_msgs.msg import CompressedImage, CameraInfo
from augmented_reality_basics import Augmenter

class AugmentedRealityBasicsNode(DTROS):

    def __init__(self, node_name):
        # Initialize DTROS parent class
        super(AugmentedRealityBasicsNode, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)
        # Get parameters from roslaunch
        self._vehicle_name = sys.argv[2]
        self._map_file = sys.argv[1]
        self.log('Retrieved roslaunch args. veh:=%s, map_file:=%s' % (self._vehicle_name, self._map_file))
        rospack = rospkg.RosPack() # To retrieve current package path
        # Read map and calibration data YAML files
        self._calib_data = self.readYamlFile('/data/config/calibrations/camera_intrinsic/' + self._vehicle_name + '.yaml')
        self.log('Loaded intrinsics calibration file')
        self._extrinsics = self.readYamlFile('/data/config/calibrations/camera_extrinsic/' + self._vehicle_name + '.yaml')
        self.log('Loaded extrinsics calibration file') 
        self._map =  self.readYamlFile(rospack.get_path('augmented_reality_basics') +'/src/maps/' + self._map_file + '.yaml')
        self.log('Loaded map')
        # Set CameraInfo Object
        self._cam_info = self.setCamInfo(self._calib_data)

        # Create CV Bridge
        self.bridge = CvBridge()
        # Create augmenter object
        self.augmenter = Augmenter(camera_info=self._cam_info, extrinsics=self._extrinsics)

        # Create subscriber to the image topic
        self.image_sub = rospy.Subscriber('/' + self._vehicle_name + '/camera_node/image/compressed', CompressedImage, self.callback)

        # Create publisher for augmented image
        self.augmented_pub = rospy.Publisher('~' + self._map_file + '/image/compressed' , CompressedImage, queue_size=1)
        self.log(node_name + ' INITIALIZED AND RUNNING')

    def readYamlFile(self,fname):
        """
        Reads the YAML file in the path specified by 'fname'.
        E.G. :
            the calibration file is located in : `/data/config/calibrations/filename/DUCKIEBOT_NAME.yaml`
        """
        with open(fname, 'r') as in_file:
            try:
                yaml_dict = yaml.load(in_file)
                return yaml_dict
            except yaml.YAMLError as exc:
                self.log("YAML syntax error. File: %s fname. Exc: %s"
                        %(fname, exc), type='fatal')
                rospy.signal_shutdown()
                return

    def setCamInfo(self, calib_data):
        cam_info = CameraInfo()
        cam_info.width = calib_data['image_width']
        cam_info.height = calib_data['image_height']
        cam_info.K = calib_data['camera_matrix']['data']
        cam_info.D = calib_data['distortion_coefficients']['data']
        cam_info.R = calib_data['rectification_matrix']['data']
        cam_info.P = calib_data['projection_matrix']['data']
        cam_info.distortion_model = calib_data['distortion_model']
        return cam_info  

    def callback(self, ros_image):

        # Convert to cv2 image using cvBridge
        image = self.bridge.compressed_imgmsg_to_cv2(ros_image)
        # Undistort image
        image = self.augmenter.process_image(image)
        # Draw the map 
        augmented_image = self.augmenter.render_segments(self._map['points'], self._map['segments'], image)
        augmented_image_msg = self.bridge.cv2_to_compressed_imgmsg(augmented_image)
        # Publish the augmented image
        self.augmented_pub.publish(augmented_image_msg)

if __name__ == '__main__':
    # create the node
    node = AugmentedRealityBasicsNode(node_name='augmented_reality_basics_node')
    # keep spinning
    rospy.spin()