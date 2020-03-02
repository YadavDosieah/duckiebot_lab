import cv2
import numpy as np
from time

#defining the color range
lower_red=np.array([160,150,0],np.uint8)
upper_red=np.array([200,255,255],np.uint8)
red_detected=False

#  variables reresenting area  and position limitation of the colors
#Red
contour_w=50
contour_h=50
red_area = 600


#Identifying the camera and opening it
cap=cv2.VideoCapture(0)
ret, frame = cap.read()

height=frame.shape[0]
width = frame.shape[1]
number_of_laps=0
while(True):
    # Capture frame-by-frame
    ret, frame = cap.read()
    hsv_image = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    #detect yellow color
    red = cv2.inRange(hsv_image, lower_red, upper_red)
    
    kernel = np.ones((5,5),np.uint8)
    redmask = cv2.dilate(red,kernel,iterations = 1)
    redmask = cv2.erode(redmask,kernel,iterations =1)
    
    clrfilter = cv2.bitwise_and(hsv_image,hsv_image,mask=redmask)
    
    cnts = cv2.findContours(red, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cnts = cnts[0] if len(cnts) == 2 else cnts[1]
    for c in cnts:
        x,y,w,h = cv2.boundingRect(c)
        if( (w>contour_w or h>contour_h) and y<height/2):
            red_detected =True;
        cv2.rectangle(frame, (x, y), (x + w, y + h), (36,255,12), 2)
    cv2.imshow('red',red)# Shows the frame
    
    if red_detected:
        print('Red is present')
        t0 = time.time()
        t1 = time.time()
        number_of_laps=number_of_laps+1
        sleep(5)
    else:
        None
        
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
    
