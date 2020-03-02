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

frame = cv2.imread('image2.png')
height ,width,channels = frame.shape


max_height =int(height/2)

croped_frame = frame[round(height/2):height,1:width]


hsv_image = cv2.cvtColor(croped_frame, cv2.COLOR_BGR2HSV)

yellow = cv2.inRange(hsv_image, y_min, y_max)
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
cv2.imshow('final slopes',croped_frame)


Trajectory_slope = (0-240)/(avr_xmin - avr_xmax) # final slope that can be used to control the vehicle dynamics

print(Trajectory_slope)




cv2.waitKey(0)
