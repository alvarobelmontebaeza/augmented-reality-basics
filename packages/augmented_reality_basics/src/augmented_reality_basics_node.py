import os
import sys
import rospy
import numpy as np
import cv2
from cv_bridge import CvBridge

import yaml
from duckietown.dtros import DTROS, NodeType
from sensor_msgs.msg import CompressedImage, CameraInfo
from augmented_reality_basics import Augmenter

class AugmentedRealityBasicsNode(DTROS):

    def __init__(self, node_name):
        # Initialize DTROS parent class
        super(AugmentedRealityBasicsNode, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)
        # Get parameters from roslaunch
        self._vehicle_name = sys.argv[1]
        self._map_file = sys.argv[2]
        # Read map and calibration data YAML files
        self._map =  self.readYamlFile('./maps/' + self.map_file + '.yaml')
        self._calib_data = self.readYamlFile('/data/config/calibrations/intrinsics/' + self._vehicle_name + '.yaml')
        # Set CameraInfo Object
        self._cam_info = self.setCamInfo(self._calib_data)

        # Create CV Bridge
        self.bridge = CvBridge()
        # Create augmenter object
        self.augmenter = Augmenter(camera_info=self._cam_info)

        # Create subscriber to the image topic
        self.image_sub = rospy.Subscriber('/' + self.vehicle_name + '/camera_node/image/compressed', CompressedImage, self.callback)

        # Create publisher for augmented image
        self.augmented_pub = rospy.Publisher('~' + self._map_file + '/image/compressed' , CompressedImage, queue_size=1)
    
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

        image = self.augmenter.process_image(image)
        
        augmented_image = self.augmenter.render_segments(self._map['points'], self._map['segments'], image)

        return augmented_image

if __name__ == '__main__':
    # create the node
    node = AugmentedRealityBasicsNode(node_name='augmented_reality_basics_node')
    # keep spinning
    rospy.spin()