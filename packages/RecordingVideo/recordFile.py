#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Mon Mar  2 17:08:46 2020

@author: ronaksharma
"""

import rospy
from duckietown import DTROS
from sensor_msgs.msg import CompressedImage
import cv2
import numpy as np

res = None
class imageProcessing(DTROS):

    def __init__(self, node_name):
    	# initialize the DTROS parent class
        super(imageProcessing, self).__init__(node_name=node_name)
    	# construct publisher
        name="/duckiebot3/camera_node/image/compressed"
        message_type=CompressedImage
        self.sub = self.subscriber(name,message_type,self.onImageReceived)
        self.fourcc = cv2.VideoWriter_fourcc(*'XVID')
        self.out = cv2.VideoWriter('/home/output.avi',self.fourcc, 20.0, (640,480))

    def onImageReceived(self,image):
        # When Image is received from topic

        np_arr = np.fromstring(image.data, np.uint8)
        frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        # write the flipped frame
        self.out.write(frame)
        
    def onShutdown(self):
        self.out.release()
        super(imageProcessing,self).onShutdown()
        
if __name__ == '__main__':
    global res
    # create the node
    node = imageProcessing(node_name='Demo3')

    # keep spinning
    rospy.spin()
