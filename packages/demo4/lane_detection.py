import numpy as np
import cv2
y_min = np.array([22,60,200],np.uint8)
y_max = np.array([60,255,255],np.uint8)
lower_white = np.array([230,220,210])
upper_white = np.array([255,255,255])

frame = cv2.imread('image4.png')
height ,width,channels = frame.shape

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
edges = cv2.Canny(gray_gaussian,200,255)
cv2.imshow('edges',edges)
cv2.waitKey(0)
