#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Sun Mar 1 11:20:02 2020

@author: YadavDosieah
"""

import os
import rospy
from duckietown import DTROS
from std_msgs.msg import String
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge

import cv2
import numpy as np

global res
res = None

class imageProcessing(DTROS):

    def __init__(self, node_name):
    	# initialize the DTROS parent class
        super(imageProcessing, self).__init__(node_name=node_name)
    	#self.image_pub = rospy.Publisher("ProcessedImage",CompressedImage,queue_size=1)
        self.led_pub = rospy.Publisher('LEDs', String, queue_size=1)

    	# construct publisher
        name="/duckiebot3/camera_node/image/compressed"
        message_type=CompressedImage
        self.sub = self.subscriber(name,message_type,self.onImageReceived)
        self.sub2 = self.subscriber('Tuning',String,self.callback)


    def onImageReceived(self,image):
        Red_Flag = False
        Yellow_Flag = False
        global res
        #defining the color range
        lower_red=np.array([160,150,0])
        upper_red=np.array([200,255,255])
        yellow_min = np.array([22,60,200])
        yellow_max = np.array([60,255,255])

        #  variables reresenting area  and position limitation of the colors
        #Yellow
        contour_w=50
        contour_h=50
        red_area = 600

        # When Image is received from topic
        np_arr = np.fromstring(image.data, np.uint8)
        frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        hsv_image = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        height = hsv_image.shape[0]
        width = hsv_image.shape[1]

        #detect Red Color
        redmask=cv2.inRange(hsv_image,lower_red,upper_red)
        kernel = np.ones((5,5),np.uint8)
        redmask = cv2.dilate(redmask,kernel,iterations = 1)
        redmask = cv2.erode(redmask,kernel,iterations =1)

        contours, hierarchy = cv2.findContours(redmask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2:]
        for plc,contour in enumerate(contours):
            area=cv2.contourArea(contour)
            if area>red_area:
                Red_Flag = True

        #detect yellow color
        yellowmask = cv2.inRange(hsv_image, yellow_min, yellow_max)
        kernel = np.ones((5,5),np.uint8)
        yellowmask = cv2.dilate(yellowmask,kernel,iterations = 1)
        yellowmask = cv2.erode(yellowmask,kernel,iterations =1)

        cnts = cv2.findContours(yellowmask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cnts = cnts[0] if len(cnts) == 2 else cnts[1]
        for c in cnts:
            x,y,w,h = cv2.boundingRect(c)
            if( (w>contour_w or h>contour_h)and x>width/2 and y>height/2):
                #Yellow is detected
                Yellow_Flag = True

        if Red_Flag:
            #print('Red color is present in the image')
            self.led_pub.publish('red line')

        if Yellow_Flag:
            #print('Yellow color is present in the image')
            self.led_pub.publish('yellow line')

        if(not (Red_Flag or Yellow_Flag)):
            self.led_pub.publish('off')

        mask = yellowmask + redmask
        res=cv2.bitwise_and(frame,frame,mask=mask)
    	cv2.imshow('yellow',res)# Shows the frame
        if cv2.waitKey(1) & 0xFF == ord('q'):
            cv2.destroyAllWindows()

    def callback(self,data):
        pass

if __name__ == '__main__':
    global res
    # create the node
    node = imageProcessing(node_name='Demo3')

    # keep spinning
    rospy.spin()
