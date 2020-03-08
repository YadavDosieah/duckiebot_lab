#!/usr/bin/env python

import os
import rospy
from duckietown import DTROS
from std_msgs.msg import String
from sensor_msgs.msg import Image
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
from duckietown_msgs.msg import WheelsCmdStamped, BoolStamped, Twist2DStamped

import cv2
import numpy as np
import math
import threading

class lapCount(DTROS):
    def __init__(self, node_name):
    	# initialize the DTROS parent class
        super(lapCount, self).__init__(node_name=node_name)
        self.led_pub = rospy.Publisher('LEDs', String, queue_size=1)

    	# construct publisher
        self.sub = self.subscriber("/duckiebot3/camera_node/image/compressed",CompressedImage,self.onImageReceived)
        self.bridge = CvBridge()
        self.image_pub = rospy.Publisher("lap_count_image_topic",Image, queue_size=1)

        #defining the color range
        rospy.set_param('~lower_red', [160,150,0])
        rospy.set_param('~upper_red', [200,255,255])
        rospy.set_param('~red_area', 10000)
        rospy.set_param('~counter_threshold', 25)
        rospy.set_param('~number_of_lap', 0)


        self.frame = None
        self.threadLock = threading.Lock()
        self.counter=0


    def onImageReceived(self,image):
        self.threadLock.acquire()
        np_arr = np.fromstring(image.data, np.uint8)
        self.frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        self.threadLock.release()


    def run(self):
        while not rospy.is_shutdown():
            number_of_lap = rospy.get_param('~number_of_lap')
            self.led_pub.publish('{}'.format(number_of_lap))
            if((number_of_lap < 3) and (self.frame is not None)):
                self.threadLock.acquire()
                frame = self.frame
                self.threadLock.release()

                lower_red=np.array(rospy.get_param('~lower_red'))
                upper_red=np.array(rospy.get_param('~upper_red'))
                red_area=rospy.get_param('~red_area')
                counter_threshold=rospy.get_param('~counter_threshold')

                height=frame.shape[0]
                width = frame.shape[1]
                red_detected=False

                #detect Red Color
                hsv=cv2.cvtColor(frame,cv2.COLOR_BGR2HSV)
                redmask=cv2.inRange(hsv, lower_red, upper_red)
                kernel = np.ones((5,5),np.uint8)
                redmask = cv2.dilate(redmask,kernel,iterations = 1)
                redmask = cv2.erode(redmask,kernel,iterations = 1)
                res=cv2.bitwise_and(frame,frame,mask=redmask)

                #median_value=cv2.medianBlur(res,15)
                #edges=cv2.Canny(median_value,100,200)
                canny2, contours, hierarchy = cv2.findContours(redmask,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)

                for plc,contour in enumerate(contours):
                    area=cv2.contourArea(contour)
                    # print(area)
                    if area>red_area:
                        red_detected=True
                    else:
                       red_detected= red_detected or False


                if red_detected:
                    self.counter=self.counter+1
                else:
                    self.counter=0

                if(self.counter==counter_threshold):
                    number_of_lap = number_of_lap + 1
                    rospy.set_param('~number_of_lap', number_of_lap)


                if(number_of_lap==3):
                    rospy.set_param('mode',3)
                    print("Mode set to 3")

                self.image_pub.publish(self.bridge.cv2_to_imgmsg(res, "bgr8"))
                # cv2.imshow('media blur',res)
                # if cv2.waitKey(1) & 0xFF == ord('q'):
                #     cv2.destroyAllWindows()

                if(self.counter > 0):
                    print(self.counter,number_of_lap)




if __name__ == '__main__':
    # create the node
    node = lapCount(node_name='demo5')
    # run node
    node.run()
    # keep spinning
    rospy.spin()
