#!/usr/bin/env python

import os
import rospy
from duckietown import DTROS
from std_msgs.msg import String
from sensor_msgs.msg import CompressedImage

import numpy as np
import cv2

low_whi_H = 220
low_whi_S = 220
low_whi_V = 210
high_whi_H = 255
high_whi_S = 255
high_whi_V = 255

low_yel_H = 22
low_yel_S = 60
low_yel_V = 200
high_yel_H = 60
high_yel_S = 255
high_yel_V = 255
final = None

class laneDetection(DTROS):

    def __init__(self, node_name):
    	# initialize the DTROS parent class
        super(laneDetection, self).__init__(node_name=node_name)
        self.led_pub = rospy.Publisher('LEDs', String, queue_size=1)

    	# construct publisher
        name="/duckiebot3/camera_node/image/compressed"
        message_type=CompressedImage
        self.sub = self.subscriber(name,message_type,self.onImageReceived)

    def onImageReceived(self,image):
        global final
        global low_whi_H
        global high_whi_H
        global low_whi_S
        global high_whi_S
        global low_whi_V
        global high_whi_V

        global low_yel_H
        global high_yel_H
        global low_yel_S
        global high_yel_S
        global low_yel_V
        global high_yel_V
        lx=[]
        ly=[]
        rx=[]
        ry=[]
        lower_white=np.array(  [low_whi_H,     low_whi_S,      low_whi_V])
        upper_white=np.array( [high_whi_H,    high_whi_S,     high_whi_V])
        yellow_min=np.array( [low_yel_H,     low_yel_S,      low_yel_V])
        yellow_max=np.array([high_yel_H,    high_yel_S,     high_yel_V])

        # When Image is received from topic
        np_arr = np.fromstring(image.data, np.uint8)
        frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        height ,width,channels = frame.shape
        max_height =int(height/2)

        croped_frame = frame[round(height/2):height,1:width]
        hsv_image = cv2.cvtColor(croped_frame, cv2.COLOR_BGR2HSV)

        yellow = cv2.inRange(hsv_image, yellow_min, yellow_max)
        white = cv2.inRange(croped_frame,lower_white,upper_white)

        clrfilter = cv2.bitwise_and(hsv_image,hsv_image,mask=yellow |white)
        color_lane = cv2.cvtColor(clrfilter,cv2.COLOR_HSV2BGR)
        gray_image = cv2.cvtColor(color_lane,cv2.COLOR_BGR2GRAY)
        gray_gaussian = cv2.GaussianBlur(gray_image,(3,3),0)
        edges = cv2.Canny(gray_gaussian,75,150)

        lines = cv2.HoughLinesP(edges,1,np.pi/180,10,maxLineGap=50,minLineLength=65)# finds the lines in the image
        #clasifies the line as leftlane line or rightlane line
        if lines is not None:
            for line in lines:
                x1, y1, x2, y2 = line[0]
                cv2.line(color_lane, (x1, y1), (x2, y2), (255, 0, 0), 2)
                if(x2-x1!=0):
                    slope = (y2-y1)/(x2-x1)
                    if slope <=0:
                        lx.extend([x1,x2])
                        ly.extend([y1,y2])
                    else:
                        rx.extend([x1,x2])
                        ry.extend([y1,y2])
        if((ly and lx and ry and rx)!= []):
            polyleft = np.poly1d(np.polyfit(ly,lx,1))
            polyright = np.poly1d(np.polyfit(ry,rx,1))

            lx_start=int(polyleft(240))
            lx_end = int(polyleft(0))
            rx_start = int(polyright(240))
            rx_end = int(polyright(0))

            final_lines = [[[lx_start,max_height,lx_end,0]],[[rx_start,max_height,rx_end,0]]]
            for lines in final_lines:
                for x1, y1, x2, y2 in lines:
                    cv2.line(croped_frame,(x1,y1),(x2,y2),[255,0,0],3)

            #these two variables provides slopes of leftlane and right lane
            avr_xmin = int((lx_start+rx_start)/2)
            avr_xmax = int((lx_end+rx_end)/2)
            cv2.line(croped_frame,(avr_xmin,max_height),(avr_xmax,0),[0,0,255],3)

            Trajectory_slope = (0-240)/(avr_xmin - avr_xmax) # final slope that can be used to control the vehicle dynamics
            print(Trajectory_slope)

        if(final is None):
            cv2.namedWindow("outputWindow", cv2.WINDOW_NORMAL);
            cv2.createTrackbar('Low White Hue', "outputWindow",            low_whi_H, 255, on_low_whi_H_thresh_trackbar)
            cv2.createTrackbar('High White Hue', "outputWindow",          high_whi_H, 255, on_high_whi_H_thresh_trackbar)
            cv2.createTrackbar('Low White Saturation', "outputWindow",     low_whi_S, 255, on_low_whi_S_thresh_trackbar)
            cv2.createTrackbar('High White Saturation', "outputWindow",   high_whi_S, 255, on_high_whi_S_thresh_trackbar)
            cv2.createTrackbar('Low White Value', "outputWindow",          low_whi_V, 255, on_low_whi_V_thresh_trackbar)
            cv2.createTrackbar('High White Value', "outputWindow",        high_whi_V, 255, on_high_whi_V_thresh_trackbar)

            cv2.createTrackbar('Low Yellow Hue', "outputWindow",         low_yel_H, 360//2, on_low_yel_H_thresh_trackbar)
            cv2.createTrackbar('High Yellow Hue', "outputWindow",       high_yel_H, 360//2, on_high_yel_H_thresh_trackbar)
            cv2.createTrackbar('Low Yellow Saturation', "outputWindow",  low_yel_S, 255, on_low_yel_S_thresh_trackbar)
            cv2.createTrackbar('High Yellow Saturation', "outputWindow",high_yel_S, 255, on_high_yel_S_thresh_trackbar)
            cv2.createTrackbar('Low Yellow Value', "outputWindow",       low_yel_V, 255, on_low_yel_V_thresh_trackbar)
            cv2.createTrackbar('High Yellow Value', "outputWindow",     high_yel_V, 255, on_high_yel_V_thresh_trackbar)

        final = np.concatenate((croped_frame, clrfilter), axis=0)
        cv2.imshow('final slopes',final)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            cv2.destroyAllWindows()


def on_low_whi_H_thresh_trackbar(val):
    global low_whi_H
    global high_whi_H
    low_whi_H = val
    low_whi_H = min(high_whi_H-1, low_whi_H)
    cv2.setTrackbarPos('Low White Hue', 'outputWindow', low_whi_H)


def on_high_whi_H_thresh_trackbar(val):
    global low_whi_H
    global high_whi_H
    high_whi_H = val
    high_whi_H = max(high_whi_H, low_whi_H+1)
    cv2.setTrackbarPos('High White Hue', 'outputWindow', high_whi_H)


def on_low_whi_S_thresh_trackbar(val):
    global low_whi_S
    global high_whi_S
    low_whi_S = val
    low_whi_S = min(high_whi_S-1, low_whi_S)
    cv2.setTrackbarPos('Low White Saturation', 'outputWindow', low_whi_S)


def on_high_whi_S_thresh_trackbar(val):
    global low_whi_S
    global high_whi_S
    high_whi_S = val
    high_whi_S = max(high_whi_S, low_whi_S+1)
    cv2.setTrackbarPos('High White Saturation', 'outputWindow', high_whi_S)


def on_low_whi_V_thresh_trackbar(val):
    global low_whi_V
    global high_whi_V
    low_whi_V = val
    low_whi_V = min(high_whi_V-1, low_whi_V)
    cv2.setTrackbarPos('Low White Value', 'outputWindow', low_whi_V)


def on_high_whi_V_thresh_trackbar(val):
    global low_whi_V
    global high_whi_V
    high_whi_V = val
    high_whi_V = max(high_whi_V, low_whi_V+1)
    cv2.setTrackbarPos('High White Value', 'outputWindow', high_whi_V)

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


def on_high_yel_V_thresh_trackbar(val):
    global low_yel_V
    global high_yel_V
    high_yel_V = val
    high_yel_V = max(high_yel_V, low_yel_V+1)
    cv2.setTrackbarPos('High Yellow Value', 'outputWindow', high_yel_V)


if __name__ == '__main__':
    # create the node
    node = laneDetection(node_name='demo4')
    # keep spinning
    rospy.spin()
