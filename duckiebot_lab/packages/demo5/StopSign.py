#!/usr/bin/env python

import os
import rospy
from duckietown import DTROS
from std_msgs.msg import String
from sensor_msgs.msg import Image
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge

import cv2
from cv2 import aruco
import numpy as np
import threading

ARUCO_PARAMETERS = aruco.DetectorParameters_create()
ARUCO_DICT = aruco.Dictionary_get(aruco.DICT_6X6_250)

class stopSign(DTROS):
    def __init__(self, node_name):
    	# initialize the DTROS parent class
        super(stopSign, self).__init__(node_name=node_name)

    	# construct publisher
        self.sub = self.subscriber("/duckiebot3/camera_node/image/compressed",CompressedImage,self.onImageReceived)

        self.frame = None
        self.threadLock = threading.Lock()
        self.prev_mode=1
        rospy.set_param('~aruco_area', 10000)


    def onImageReceived(self,image):
        self.threadLock.acquire()
        np_arr = np.fromstring(image.data, np.uint8)
        self.frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        self.threadLock.release()


    def run(self):
        while not rospy.is_shutdown():
            mode = rospy.get_param('mode')
            if(mode==1 or mode ==2):
                self.prev_mode = mode
            if(self.frame is not None):
                self.threadLock.acquire()
                frame = self.frame
                self.threadLock.release()
                # grayscale image
                gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

                # Detect Aruco markers
                corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, ARUCO_DICT, parameters=ARUCO_PARAMETERS)

                if corners != []:
                    aruco_area = rospy.get_param('~aruco_area')
                    #print("Aruco Detected")
                    c = corners[0][0]

                    centre = (c[:,0].mean(),c[:,1].mean())
                    if(PolyArea(c[:,0],c[:,1]) > aruco_area):
                        rospy.set_param('mode',3)
                        print("Mode set to 3")
                    else:
                        rospy.set_param('mode',self.prev_mode)
                        print("Mode set to {}".format(self.prev_mode))

                    # # Outline all of the markers detected in our image
                    # frame = aruco.drawDetectedMarkers(frame, corners, borderColor=(0, 0, 255))
                    # cv2.circle(frame, (int(centre[0]),int(centre[1])), 5, ( 255, 0, 0 ), -1, 8 )

                else:
                    rospy.set_param('mode',self.prev_mode)
                    print("Mode set to {}".format(self.prev_mode))

                # cv2.imshow('outputWindow', frame)
                # if cv2.waitKey(1) & 0xFF == ord('q'):
                #     cv2.destroyAllWindows()

def PolyArea(x,y):
    return 0.5*np.abs(np.dot(x,np.roll(y,1))-np.dot(y,np.roll(x,1)))

if __name__ == '__main__':
    # create the node
    node = stopSign(node_name='demo5_stopSign')
    # run node
    node.run()
    # keep spinning
    rospy.spin()
