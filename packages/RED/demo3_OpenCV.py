#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Fri Feb 28 11:00:28 2020

@author: ronaksharma
"""
import cv2
import numpy as np
from PIL import Image, ImageEnhance

cap=cv2.VideoCapture(0)
#cap = cv2.VideoCapture('test.mp4')

cap.set(3,320)
cap.set(4,240)

scale=2
#HSV better than RGB, Each value in HSV are format, but in RGB these are dependent. V- Color Value, S - Satuation like intensity and H - hue -This dictates the color
# This function are responsible for filtering colour
#
lower_red=np.array([160,150,0])    
upper_red=np.array([200,255,255])
    
# You can apply filters which literally just blurs the images, thus ignoring any distrurbances

factor_contrast =  1 #CHANG FOR CONTRAST
factor_brightness = 1  #CHANGE FOR BRIGHTNESS
factor_sharpness =  1 #CHANGE FOR SHARPNESS


def changeFrameColour(lower_value,upper_value):
    hsv=cv2.cvtColor(frame,cv2.COLOR_BGR2HSV)
    mask=cv2.inRange(hsv,lower_value,upper_value)
    res=cv2.bitwise_and(frame,frame,mask=mask)
    median_value=cv2.medianBlur(res,15)
    return median_value
    
def areaCalculate(median_value):
    edges=cv2.Canny(median_value,100,200)
    canny2, contours, hierarchy = cv2.findContours(edges,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
    for plc,contour in enumerate(contours):
        area=cv2.contourArea(contour)
        if area>400:
            return 1
        else:
            return 2

def shapeIdentifier(median_value,name_tri,name_rectangle):
        edges=cv2.Canny(median_value,100,200)
        canny2, contours, hierarchy = cv2.findContours(edges,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)

        for i in range(0,len(contours)):
            #approximate the contour with accuracy proportional to
            #the contour perimeter
            approx = cv2.approxPolyDP(contours[i],cv2.arcLength(contours[i],True)*0.105,True)
            #Skip small or non-convex objects
            if(abs(cv2.contourArea(contours[i]))<300 or not(cv2.isContourConvex(approx))):
                continue

            x,y,w,h = cv2.boundingRect(contours[i])
            cv2.putText(frame,name_tri,(x,y),cv2.FONT_HERSHEY_SIMPLEX,scale,(255,255,0),2,cv2.LINE_AA)
            cv2.rectangle(frame,(x,y),(x+w,y+h),(255,0,0),2)
            cv2.imshow('edges',edges)
            #cv2.imshow('m',median_value)


     
while True:
    _,frame=cap.read() #'_' is a value returned to this function, but we don't care about that value
    
#    frame=Image.fromarray(frame)
#    enhancer = ImageEnhance.Contrast(frame)
#    image=enhancer.enhance(factor_contrast)
#    #frame = np.array(image).reshape((image.height, image.width,3))
#    
#    enhancer=ImageEnhance.Brightness(image)
#    image=enhancer.enhance(factor_brightness)
#    frame = np.array(image).reshape((image.height, image.width,3))
#    
#    enhancer=ImageEnhance.Sharpness(image) 
#    image=enhancer.enhance(factor_sharpness)
#    frame = np.array(image).reshape((image.height, image.width,3))
   
    
    median_red=changeFrameColour(lower_red,upper_red)
    area_red=areaCalculate(median_red)
    
    #Using elif instead of multiple If's as the program should detect only one colour at a time
    if(area_red==1): #MAKE THE CONDITION THAT ONLY DETECT ONE COLOUR  AT TIME , X==1 AND Y!=1 AND Z!=1
        median_value=median_red
        shapeIdentifier(median_value,'','')
        cv2.imshow('m',median_red)
    else:
        None
        
    cv2.imshow('frame',frame)
    k = cv2.waitKey(1) & 0xFF
    # press 'q' to exit

cap.release()
cv2.destroyAllWindows()