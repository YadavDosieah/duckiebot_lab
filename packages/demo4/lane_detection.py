import numpy as np
import cv2
lx=[]
ly=[]
rx=[]
ry=[]
y_min = np.array([22,60,200],np.uint8)
y_max = np.array([60,255,255],np.uint8)
lower_white = np.array([230,220,210])
upper_white = np.array([255,255,255])

frame = cv2.imread('image4.png')
height ,width,channels = frame.shape

min_height = height/2
max_height =height

croped_frame = frame[round(height/2):height,1:width]

cv2.imshow('croped_image',croped_frame)
hsv_image = cv2.cvtColor(croped_frame, cv2.COLOR_BGR2HSV)
cv2.imshow('hs image',hsv_image)
yellow = cv2.inRange(hsv_image, y_min, y_max)
white = cv2.inRange(croped_frame,lower_white,upper_white)

clrfilter = cv2.bitwise_and(hsv_image,hsv_image,mask=yellow |white)
color_lane = cv2.cvtColor(clrfilter,cv2.COLOR_HSV2BGR)
gray_image = cv2.cvtColor(color_lane,cv2.COLOR_BGR2GRAY)
gray_gaussian = cv2.GaussianBlur(gray_image,(3,3),0)
edges = cv2.Canny(gray_gaussian,75,150)
cv2.imshow('edges',edges)
lines = cv2.HoughLinesP(edges,1,np.pi/180,10,maxLineGap=50)# finds the lines in the image

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



cv2.imshow('left',gray_image)
polyleft = np.poly1d(np.polyfit(ly,lx,1))
polyright = np.poly1d(np.polyfit(ry,rx,1))
print(polyleft)
print(polyright)
lx_start=int(polyleft(max_height))
lx_end = int(polyleft(min_height))
rx_start = int(polyright(max_height))
rx_end = int(polyright(min_height))

final_lines = [[[lx_start,int(max_height),lx_end,int(min_height)],[rx_start,int(max_height),rx_end,int(min_height)]]]
for lines in final_lines:
    for x1,y1,x2,y2 in line:
        cv2.line(croped_frame,(x1,y1),(x2,y2),[255,0,0],3)
cv2.imshow('lines',color_lane)
cv2.imshow('final slopes',croped_frame)
cv2.waitKey(0)
