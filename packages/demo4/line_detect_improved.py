#!/usr/bin/env python

import os
import rospy
from duckietown import DTROS
from std_msgs.msg import String
from sensor_msgs.msg import Image
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
from duckietown_msgs.msg import WheelsCmdStamped, BoolStamped, Twist2DStamped

import math
import numpy as np
import cv2

class laneDetection(DTROS):

    def __init__(self, node_name):
    	# initialize the DTROS parent class
        super(laneDetection, self).__init__(node_name=node_name)
        self.led_pub = rospy.Publisher('LEDs', String, queue_size=1)
        self.wheel_pub = rospy.Publisher('/duckiebot3/wheels_driver_node/wheels_cmd', WheelsCmdStamped, queue_size=1)
        self.car_pub = rospy.Publisher('/duckiebot3/wheels_driver_node/car_cmd', Twist2DStamped, queue_size=1)
        self.line_detection_pub = rospy.Publisher('line_detection', String, queue_size=1)

    	# construct publisher
        self.sub = self.subscriber("/duckiebot3/camera_node/image/compressed",CompressedImage,self.onImageReceived)
        self.bridge = CvBridge()

        self.image_pub = rospy.Publisher("image_topic",Image, queue_size=1)
        self.frame = None

        self.maxangle = 0    #max angle of average line found
        self.width = 0   #width of image
        self.yellow_x = 0    #x-axis location of yellow line, used to ignore white lines on wrong side

        #Stores previous yellow line found in case yellow line cannot be found (sometimes for a frame or 2 on corners)
        self.yellow_line_prev = []
        self.yellow_start_prev = 0
        self.yellow_end_prev = 0

        #Stores previous white line found in case white line cannot be found (sometimes for a frame or 2 on corners)
        self.white_line_prev = []
        self.white_start_prev = 0
        self.white_end_prev = 0

        rospy.set_param('~y_min', [22, 60, 200])
        rospy.set_param('~y_max', [100, 255, 255])

        rospy.set_param('~lower_white', [230, 220, 210])
        rospy.set_param('~upper_white', [255, 255, 255])

        #Change this value to look at a certain percentage of full image measured from bottom
        rospy.set_param('~max_height_percent', 0.6)

        rospy.set_param('~maxLineGap',50)
        rospy.set_param('~minLineLength',50)
        rospy.set_param('~threshold',25)

        rospy.set_param('mode',1)

        rospy.set_param("~max_angle_change",25)
        rospy.set_param("~speed",0.2)


    def onShutdown(self):
        self.stop()
        super(laneDetection,self).onShutdown()


    def onImageReceived(self,image):
        np_arr = np.fromstring(image.data, np.uint8)
        self.frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)


    #Processes individual frames/images
    def run(self):
        while not rospy.is_shutdown():
            if(self.frame is not None):
                frame = self.frame

                #Min and max thresholds for yellow mask (HSV)
                y_min = np.array(rospy.get_param("~y_min"))
                y_max = np.array(rospy.get_param("~y_max"))

                #Min and max thresholds for white mask (HSV)
                lower_white = np.array(rospy.get_param("~lower_white"))
                upper_white = np.array(rospy.get_param("~upper_white"))

                max_height_percent = rospy.get_param("~max_height_percent")

                height, self.width, channels = frame.shape
                max_height = height * max_height_percent

                croped_frame = frame[round(height * (1-max_height_percent)):round(height*1), 1:self.width]
                hsv_image = cv2.cvtColor(croped_frame, cv2.COLOR_RGB2HSV)


                yellow = cv2.inRange(hsv_image, y_min, y_max)
                #Dilate and erode yellow to form one solid object from dashed line
                kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (35, 35))
                yellow_morph = cv2.dilate(yellow, kernel, iterations=1)
                kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (15, 15))
                yellow_morph = cv2.erode(yellow_morph, kernel, iterations=1)

                white = cv2.inRange(croped_frame, lower_white, upper_white)


                yellow_line, yellow_start, yellow_end = self.hough(yellow_morph, max_height, 'yellow')   #Identify location of yellow line
                white_line, white_start, white_end = self.hough(white, max_height, 'white')  #Identify location of white line

                cv2.line(croped_frame, (int(white_start), int(max_height)), (int(white_end), int(0)), [255, 0, 0], 3)
                cv2.line(croped_frame, (int(yellow_start), int(max_height)), (int(yellow_end), int(0)), [255, 0, 0], 3)

                #Calculate line half way between yellow and white
                avr_xmin = int((white_start + yellow_start) / 2)
                avr_xmax = int((white_end + yellow_end) / 2)
                cv2.line(croped_frame, (int(avr_xmin), int(max_height)), (int(avr_xmax), 0), [0, 0, 255], 3)

                if (avr_xmin - avr_xmax != 0):
                    Trajectory_slope = (0 - 240) / (
                            avr_xmin - avr_xmax)  # Final slope that can be used to control the vehicle dynamics
                    angle = math.atan(Trajectory_slope)
                    angle = math.degrees(angle)

                    if angle < 0:
                        angle = -90 - angle
                    else:
                        angle = 90 - angle
                else:
                    angle = 0

                if (abs(angle) > self.maxangle):
                    self.maxangle = abs(angle)
                    self.line_detection_pub.publish('New Max Angle: {}'.format(self.maxangle))
                self.line_detection_pub.publish('Angle: {}'.format(angle))

                mode = rospy.get_param("mode")

                if(mode == 1):
                    self.move_kin(angle)
                elif(mode == 2):
                    self.move_wheel(angle)
                else:
                    self.stop()

                self.image_pub.publish(self.bridge.cv2_to_imgmsg(croped_frame, "bgr8"))
                # cv2.imshow('croped_frame',croped_frame)
                # if cv2.waitKey(1) & 0xFF == ord('q'):
                #     cv2.destroyAllWindows()


    def move_wheel(self,angle):
        max_angle_change = rospy.get_param("~max_angle_change")
        speed = rospy.get_param("~speed")

        if angle >= 0:
            vel_left = (1 - angle / max_angle_change) * speed
            vel_right = speed
        else:
            vel_left = speed
            vel_right = (1 - abs(angle) / max_angle_change) * speed

        msg = WheelsCmdStamped()
        msg.header.stamp = rospy.get_rostime()
        msg.vel_left = vel_left
        msg.vel_right = vel_right
        self.wheel_pub.publish(msg)

        self.line_detection_pub.publish('Left: {}'.format(vel_left))
        self.line_detection_pub.publish('Right: {}'.format(vel_right))


    def move_kin(self,angle):
        speed = rospy.get_param("~speed")
        msg = Twist2DStamped()
        msg.header.stamp = rospy.get_rostime()
        msg.v = speed
        msg.omega = angle
        self.car_pub.publish(msg)


    def stop(self):
        msg = WheelsCmdStamped()
        msg.header.stamp = rospy.get_rostime()
        msg.vel_left = 0
        msg.vel_right = 0
        self.wheel_pub.publish(msg)


    def hough(self,img, max_height, colour):
        x = []  #X coordinates of line
        y = []  #Y coordinates of line

        #Used to store hough line found closest to roadway as yellow and white lines are quite thick and return multiple hough lines
        x1_max = self.width
        x2_max = self.width
        x1_min = 0
        x2_min = 0

        max_Gap = rospy.get_param("~maxLineGap")
        min_Length = rospy.get_param("~minLineLength")
        threshold = rospy.get_param("~threshold")

        edges = cv2.Canny(img, 75, 150) #Edge detection
        lines = cv2.HoughLinesP(edges, 1, np.pi / 180, threshold, maxLineGap=max_Gap,
                                minLineLength=min_Length)  #Finds the lines in the image

        if lines is not None:
            for line in lines:
                x1, y1, x2, y2 = line[0]
                if colour == 'white':
                    if (x1 > x1_min and x2 > x2_min) and (x1 < self.yellow_x and x2 < self.yellow_x): #If line found is closer to roadway than previous closest line and if it is on correct side of yellow line
                        m = (y2-y1)/(x2-x1) #Gradient of line
                        if m  > 0.5 or m < -0.5:    #Ignore lines with low gradient as they are probably horizontal lines that we don't want
                            x = [x1, x2]
                            y = [y1, y2]
                            final_line =[[[x1, y1, x2, y2]]]
                            self.white_line_prev = final_line    #Store new line as previous line for next frame
                            x1_min = x1
                            x2_min = x2
                else:
                    if x1 < x1_max and x2 < x2_max:
                        m = (y2-y1)/(x2-x1)
                        if m  > 0.5 or m < -0.5:
                            x = [x1, x2]
                            y = [y1, y2]
                            final_line = [[[x1, y1, x2, y2]]]
                            self.yellow_line_prev = final_line
                            self.yellow_x = x1
                            x1_max = x1
                            x2_max = x2
                cv2.line(edges, (int(x1), int(y1)), (int(x2), int(y2)), (255, 0, 0), 2)
        if (x and y) != []: #If line has been found
            poly = np.poly1d(np.polyfit(y,x,1)) #Find equation of line

            #Find coordinates so line is drawn across entire image
            start = int(poly(max_height))
            end = int(poly(0))


            if colour == 'white':
                self.white_start_prev = start
                self.white_end_prev = end
            else:
                self.yellow_start_prev = start
                self.yellow_end_prev = end
        elif colour == 'white': #If line has not been found use line found in last good frame
            self.line_detection_pub.publish('WARNING: PREVIOUS CALC')
            final_line = self.white_line_prev
            start = self.white_start_prev
            end = self.white_end_prev
        else:
            self.line_detection_pub.publish('WARNING: PREVIOUS CALC')
            final_line = self.yellow_line_prev
            start = self.yellow_start_prev
            end = self.yellow_end_prev

        return final_line, start, end


if __name__ == '__main__':
    # create the node
    node = laneDetection(node_name='demo4')
    # run node
    node.run()
    # keep spinning
    rospy.spin()
