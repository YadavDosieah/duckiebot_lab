import cv2
import numpy as np
import math
import time
#defining the color range
lower_red=np.array([160,150,0],np.uint8)
upper_red=np.array([200,255,255],np.uint8)
red_detected=False

#  variables reresenting area  and position limitation of the colors
#Red
red_area = 200

perLapInformation=0
#Identifying the camera and opening it
cap=cv2.VideoCapture(0)
ret, frame = cap.read()

height=frame.shape[0]
width = frame.shape[1]

time_redNotDetected=time.time()
time_redDetected=time.time()

lapDetected=0
number_of_lap=0
counter=0

while(True):
    # Capture frame-by-frame
    #      kernel = np.ones((5,5),np.uint8)
#    redmask = cv2.dilate(red,kernel,iterations = 1)
#    redmask = cv2.erode(redmask,kernel,iterations =1)
    ret, frame = cap.read()

    hsv=cv2.cvtColor(frame,cv2.COLOR_BGR2HSV)
    mask=cv2.inRange(hsv, lower_red, upper_red)
    res=cv2.bitwise_and(frame,frame,mask=mask)
    median_value=cv2.medianBlur(res,15)
    edges=cv2.Canny(median_value,100,200)
    canny2, contours, hierarchy = cv2.findContours(edges,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)

    for plc,contour in enumerate(contours):
        area=cv2.contourArea(contour)
        if area>red_area:
            red_detected=True
        else:
           red_detected=False
    cv2.imshow('red',median_value)# Shows the frame
    
    if red_detected:
        counter=counter+1
    else:
        counter=0
    
    if(counter==50):
        number_of_lap=number_of_lap+1
        
    
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
        
    print(counter,number_of_lap)
    
