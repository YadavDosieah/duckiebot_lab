#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sun Feb 23 10:44:02 2020

@author: ronaksharma
"""
import os
import rospy
from duckietown import DTROS
from std_msgs.msg import String
import cv2
import numpy as np
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge

class imageProcessing(DTROS):
    
    def __init__(self, node_name):
        # initialize the DTROS parent class
        super(imageProcessing, self).__init__(node_name=node_name)
        path="/output/image_raw/compressed"
        self.image_pub = rospy.Publisher(path,CompressedImage)
        
        # construct publisher
        name="/duckiebot3/camera_node/image/compressed"
        message_type=CompressedImage
        self.sub = self.subscriber(name,message_type,self.onImageReceived(self,CompressedImage))
        
    def onImageReceived(self,image):
        np_arr = np.fromstring(image.data, np.uint8)
        image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        
        img=cv2.cvtColor(image_np, cv2.COLOR_BGR2GRAY)
#        image_message = CvBridge.cv2_to_imgmsg(cv_image, encoding="bgr8")

        #### Create CompressedIamge ####
        msg = CompressedImage()
        msg.header.stamp = rospy.Time.now()
        msg.format = "jpeg"
        msg.data = np.array(cv2.imencode('.jpg', img)[1]).tostring()
        # Publish new image
        self.image_pub.publish(msg)
        

if __name__ == '__main__':
    # create the node
    node = imageProcessing(node_name='imageProcessing')
    # run node
    node.onImageReceived()
