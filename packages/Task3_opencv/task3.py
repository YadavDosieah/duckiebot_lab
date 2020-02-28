import cv2
import numpy as np

#defining the color range
y_min = np.array([22,60,200],np.uint8)
y_max = np.array([60,255,255],np.uint8)

lower_red=np.array([160,150,0])
upper_red=np.array([200,255,255])

#  variables reresenting area  and position limitation of the colors
#Yellow
contour_w=50
contour_h=50
image_middle = 200
yellow_detected =False


#Red
lower_red=np.array([160,150,0])
upper_red=np.array([200,255,255])
red_area = 400
red_detected = False


#Identifying the camera and opening it
cap=cv2.VideoCapture(0)
# provides the resolution of the video
height =cap.get(4)
width = cap.get(3)


while(True):
    # Capture frame-by-frame
    ret, frame = cap.read()
    hsv_image = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    
    #detect Red Color
    mask=cv2.inRange(hsv_image,lower_red,upper_red)
    res=cv2.bitwise_and(frame,frame,mask=mask)
    median_value=cv2.medianBlur(res,15)
    edges=cv2.Canny(median_value,100,200)
    contours, hierarchy = cv2.findContours(mask,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
    for plc,contour in enumerate(contours):
        area=cv2.contourArea(contour)
        if area>red_area:
            red_detected = True
    if red_detected:
        print('Red color is present in the image')
        red_detected=False
    else:
        print('no Red in the image')

    #detect yellow color
    yellow = cv2.inRange(hsv_image, y_min, y_max)
    clrfilter = cv2.bitwise_and(hsv_image,hsv_image,mask=yellow)# masks yellow out
    cnts = cv2.findContours(yellow, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cnts = cnts[0] if len(cnts) == 2 else cnts[1]
    for c in cnts:
        x,y,w,h = cv2.boundingRect(c)
        if( (w>contour_w or h>contour_h)and x>width/2 and y>height/2):
            yellow_detected =True;

        cv2.rectangle(frame, (x, y), (x + w, y + h), (36,255,12), 2)
    cv2.imshow('yellow',res)# Shows the frame


    if yellow_detected:
        print('Yellow is present')
        yellow_detected=False
    else:
        print('No yellow in the image')
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
