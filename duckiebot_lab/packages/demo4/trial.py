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
from duckietown_msgs.msg import WheelsCmdStamped, BoolStamped, Twist2DStamped
import cv2
import numpy as np

global res
<<<<<<< HEAD
global low_red_H
global low_red_S
global low_red_V
global high_red_H
global high_red_S
global high
=======
>>>>>>> 1cd57bc533754d8fa2f165b7445469216b8f7559
low_red_H = 160
low_red_S = 150
low_red_V = 0
high_red_H = 200
high_red_S = 255
high_red_V = 255

low_yel_H = 22
low_yel_S = 60
low_yel_V = 200
high_yel_H = 60
high_yel_S = 255
high_yel_V = 255
res = None

class imageProcessing(DTROS):

    def __init__(self, node_name):
    	# initialize the DTROS parent class
        super(imageProcessing, self).__init__(node_name=node_name)
    	#self.image_pub = rospy.Publisher("ProcessedImage",CompressedImage,queue_size=1)
        self.led_pub = rospy.Publisher('LEDs', String, queue_size=1)
<<<<<<< HEAD
        self.wheel_pub = rospy.Publisher('/duckiebot3/wheels_driver_node/wheels_cmd', WheelsCmdStamped, queue_size=1)
        self.car_pub = rospy.Publisher('/duckiebot3/wheels_driver_node/car_cmd', Twist2DStamped, queue_size=1)
        self.line_detection_pub = rospy.Publisher('line_detection', String, queue_size=1)
=======
>>>>>>> 1cd57bc533754d8fa2f165b7445469216b8f7559

    	# construct publisher
        name="/duckiebot3/camera_node/image/compressed"
        message_type=CompressedImage
        self.sub = self.subscriber(name,message_type,self.onImageReceived)


    def onImageReceived(self,image):
        Red_Flag = False
        Yellow_Flag = False
        global res
        global low_red_H
        global high_red_H
        global low_red_S
        global high_red_S
        global low_red_V
        global high_red_V

        global low_yel_H
        global high_yel_H
        global low_yel_S
        global high_yel_S
        global low_yel_V
        global high_yel_V
        #defining the color range
        lower_red=np.array(  [low_red_H,     low_red_S,      low_red_V])
        upper_red=np.array( [high_red_H,    high_red_S,     high_red_V])
        yellow_min=np.array( [low_yel_H,     low_yel_S,      low_yel_V])
        yellow_max=np.array([high_yel_H,    high_yel_S,     high_yel_V])

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
            if( (w>contour_w or h>contour_h)and x>width/4 and y>height/2):
                #Yellow is detected
                Yellow_Flag = True

        if Red_Flag:
            #print('Red color is present in the image')
            self.led_pub.publish('red line')

        if Yellow_Flag:
            #print('Yellow color is present in the image')
            self.led_pub.publish('yellow line')
            self.move_wheel(2)
        else:
            self.move_wheel(1)
<<<<<<< HEAD

=======
            
>>>>>>> 1cd57bc533754d8fa2f165b7445469216b8f7559

        if(not (Red_Flag or Yellow_Flag)):
            self.led_pub.publish('off')

        #if(res is None):
            #cv2.namedWindow("outputWindow", cv2.WINDOW_NORMAL);
            #cv2.createTrackbar('Low Red Hue', "outputWindow",            low_red_H, 360//2, on_low_red_H_thresh_trackbar)
            #cv2.createTrackbar('High Red Hue', "outputWindow",          high_red_H, 360//2, on_high_red_H_thresh_trackbar)
            #cv2.createTrackbar('Low Red Saturation', "outputWindow",     low_red_S, 255, on_low_red_S_thresh_trackbar)
            #cv2.createTrackbar('High Red Saturation', "outputWindow",   high_red_S, 255, on_high_red_S_thresh_trackbar)
            #cv2.createTrackbar('Low Red Value', "outputWindow",          low_red_V, 255, on_low_red_V_thresh_trackbar)
            #cv2.createTrackbar('High Red Value', "outputWindow",        high_red_V, 255, on_high_red_V_thresh_trackbar)

            #cv2.createTrackbar('Low Yellow Hue', "outputWindow",         low_yel_H, 360//2, on_low_yel_H_thresh_trackbar)
            #cv2.createTrackbar('High Yellow Hue', "outputWindow",       high_yel_H, 360//2, on_high_yel_H_thresh_trackbar)
            #cv2.createTrackbar('Low Yellow Saturation', "outputWindow",  low_yel_S, 255, on_low_yel_S_thresh_trackbar)
            #cv2.createTrackbar('High Yellow Saturation', "outputWindow",high_yel_S, 255, on_high_yel_S_thresh_trackbar)
            #cv2.createTrackbar('Low Yellow Value', "outputWindow",       low_yel_V, 255, on_low_yel_V_thresh_trackbar)
            #cv2.createTrackbar('High Yellow Value', "outputWindow",     high_yel_V, 255, on_high_yel_V_thresh_trackbar)
        # mask = yellowmask + redmask
        #red=cv2.bitwise_and(frame,frame,mask=redmask)
        #yellow=cv2.bitwise_and(frame,frame,mask=yellowmask)
        #res = np.concatenate((red, yellow), axis=1)
        #cv2.imshow('ProcessedImage',res)# Shows the frame
        #if cv2.waitKey(1) & 0xFF == ord('q'):
            #cv2.destroyAllWindows()


def on_low_red_H_thresh_trackbar(val):
    global low_red_H
    global high_red_H
    low_red_H = val
    low_red_H = min(high_red_H-1, low_red_H)
    cv2.setTrackbarPos('Low Red Hue', 'outputWindow', low_red_H)


def on_high_red_H_thresh_trackbar(val):
    global low_red_H
    global high_red_H
    high_red_H = val
    high_red_H = max(high_red_H, low_red_H+1)
    cv2.setTrackbarPos('High Red Hue', 'outputWindow', high_red_H)


def on_low_red_S_thresh_trackbar(val):
    global low_red_S
    global high_red_S
    low_red_S = val
    low_red_S = min(high_red_S-1, low_red_S)
    cv2.setTrackbarPos('Low Red Saturation', 'outputWindow', low_red_S)


def on_high_red_S_thresh_trackbar(val):
    global low_red_S
    global high_red_S
    high_red_S = val
    high_red_S = max(high_red_S, low_red_S+1)
    cv2.setTrackbarPos('High Red Saturation', 'outputWindow', high_red_S)


def on_low_red_V_thresh_trackbar(val):
    global low_red_V
    global high_red_V
    low_red_V = val
    low_red_V = min(high_red_V-1, low_red_V)
    cv2.setTrackbarPos('Low Red Value', 'outputWindow', low_red_V)


def on_high_red_V_thresh_trackbar(val):
    global low_red_V
    global high_red_V
    high_red_V = val
    high_red_V = max(high_red_V, low_red_V+1)
    cv2.setTrackbarPos('High Red Value', 'outputWindow', high_red_V)

def on_low_yel_H_thresh_trackbar(val):
    global low_yel_H
    global high_yel_H
    low_yel_H = val
    low_yel_H = min(high_yel_H-1, low_yel_H)
    cv2.setTrackbarPos('Low Yellow Hue', 'outputWindow', low_yel_H)


def on_high_yel_H_thresh_trackbar(val):
    global low_yel_H
    global high_yel_H
    high_yel_H = val
    high_yel_H = max(high_yel_H, low_yel_H+1)
    cv2.setTrackbarPos('High Yellow Hue', 'outputWindow', high_yel_H)


def on_low_yel_S_thresh_trackbar(val):
    global low_yel_S
    global high_yel_S
    low_yel_S = val
    low_yel_S = min(high_yel_S-1, low_yel_S)
    cv2.setTrackbarPos('Low Yellow Saturation', 'outputWindow', low_yel_S)


def on_high_yel_S_thresh_trackbar(val):
    global low_yel_S
    global high_yel_S
    high_yel_S = val
    high_yel_S = max(high_yel_S, low_yel_S+1)
    cv2.setTrackbarPos('High Yellow Saturation', 'outputWindow', high_yel_S)


def on_low_yel_V_thresh_trackbar(val):
    global low_yel_V
    global high_yel_V
    low_yel_V = val
    low_yel_V = min(high_yel_V-1, low_yel_V)
    cv2.setTrackbarPos('Low Yellow Value', 'outputWindow', low_yel_V)


<<<<<<< HEAD
def on_high_yel_V_thresh_trackbar(val)line_detection_pub:
=======
def on_high_yel_V_thresh_trackbar(val):
>>>>>>> 1cd57bc533754d8fa2f165b7445469216b8f7559
    global low_yel_V
    global high_yel_V
    high_yel_V = val
    high_yel_V = max(high_yel_V, low_yel_V+1)
    cv2.setTrackbarPos('High Yellow Value', 'outputWindow', high_yel_V)

def move_wheel(self,flag):
        speed = rospy.get_param("~speed")
        if flag==1:
            vel_left = speed
            vel_right = speed/3
        else:
            vel_left =  speed/3
            vel_right = speed
<<<<<<< HEAD

=======
      
>>>>>>> 1cd57bc533754d8fa2f165b7445469216b8f7559

        msg = WheelsCmdStamped()
        msg.header.stamp = rospy.get_rostime()
        msg.vel_left = vel_left
        msg.vel_right = vel_right
        self.wheel_pub.publish(msg)

        self.line_detection_pub.publish('Left: {}'.format(vel_left))
        self.line_detection_pub.publish('Right: {}'.format(vel_right))



if __name__ == '__main__':
    # create the node
    node = imageProcessing(node_name='Demo3')
    # keep spinning
<<<<<<< HEAD
    rospy.spin()
=======
    rospy.spin()
>>>>>>> 1cd57bc533754d8fa2f165b7445469216b8f7559
