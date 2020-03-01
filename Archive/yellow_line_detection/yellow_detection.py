import numpy as np
import cv2
y_min = np.array([22,60,200],np.uint8)
y_max = np.array([60,255,255],np.uint8)
contour_w=100
contour_h=100
image_middle = 200
cap = cv2.VideoCapture(0)
height =cap.get(4)
width = cap.get(3)

while(True):
    # Capture frame-by-frame
    ret, frame = cap.read()
    hsv_image = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    yellow = cv2.inRange(hsv_image, y_min, y_max)
    clrfilter = cv2.bitwise_and(hsv_image,hsv_image,mask=yellow)
    cnts = cv2.findContours(yellow, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cnts = cnts[0] if len(cnts) == 2 else cnts[1]
    for c in cnts:
        x,y,w,h = cv2.boundingRect(c)
        if((w>contour_w or h>contour_h) and x>width/2):
            print('found yellow at')
        cv2.rectangle(frame, (x, y), (x + w, y + h), (36,255,12), 2)
    cv2.imshow('yellow',frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# When everything done, release the capture

cap.release()
cv2.destroyAllWindows()
cd
