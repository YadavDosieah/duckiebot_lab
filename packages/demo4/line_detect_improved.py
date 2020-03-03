
import math
import numpy as np
import cv2
import matplotlib.pyplot as plt

from moviepy.editor import VideoFileClip

maxangle = 0    #max angle of average line found
width = 0   #width of image
yellow_x = 0    #x-axis location of yellow line, used to ignore white lines on wrong side
max_height = 0  #height of cropped image

#Stores previous yellow line found in case yellow line cannot be found (sometimes for a frame or 2 on corners)
yellow_line_prev = []
yellow_start_prev = 0
yellow_end_prev = 0

#Stores previous white line found in case white line cannot be found (sometimes for a frame or 2 on corners)
white_line_prev = []
white_start_prev = 0
white_end_prev = 0

#Processes individual frames/images
def process(frame):
    global maxangle
    global width
    global max_height

    #Min and max thresholds for yellow mask (HSV)
    y_min = np.array([22, 60, 200], np.uint8)
    y_max = np.array([100, 255, 255], np.uint8)

    #Min and max thresholds for white mask (HSV)
    lower_white = np.array([230, 220, 210])
    upper_white = np.array([255, 255, 255])

    height, width, channels = frame.shape

    max_height_percent = 0.6    #Change this value to look at a certain percentage of full image measured from bottom

    max_height = height * max_height_percent

    croped_frame = frame[round(height * (1-max_height_percent)):round(height*1), 1:width]

    croped_frame = cv2.cvtColor(croped_frame, cv2.COLOR_BGR2RGB)



    hsv_image = cv2.cvtColor(croped_frame, cv2.COLOR_RGB2HSV)

    yellow = cv2.inRange(hsv_image, y_min, y_max)

    #Dilate and erode yellow to form one solid object from dashed line
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (35, 35))
    yellow_morph = cv2.dilate(yellow, kernel, iterations=1)
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (15, 15))
    yellow_morph = cv2.erode(yellow_morph, kernel, iterations=1)

    white = cv2.inRange(croped_frame, lower_white, upper_white)

    yellow_line, yellow_start, yellow_end = hough(yellow_morph, max_height, 'yellow')   #Identify location of yellow line
    white_line, white_start, white_end = hough(white, max_height, 'white')  #Identify location of white line


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

    if (abs(angle) > maxangle):
        maxangle = abs(angle)
        print('New Max Angle: ', maxangle)
    print('Angle: ', angle)
    move(angle)
    croped_frame = cv2.cvtColor(croped_frame, cv2.COLOR_BGR2RGB)
    return croped_frame

def move(angle):
    if angle >= 0:
        vel_left = (1 - angle / 50) * 0.2
        vel_right = 0.2
    else:
        vel_left = 0.2
        vel_right = (1 - abs(angle) / 50) * 0.2

    print('Left: ', vel_left)
    print('Right: ', vel_right)


def hough(img, max_height, colour):
    global width
    global yellow_x

    global yellow_line_prev
    global yellow_start_prev
    global yellow_end_prev
    global white_line_prev
    global white_start_prev
    global white_end_prev

    x = []  #X coordinates of line
    y = []  #Y coordinates of line

    #Used to store hough line found closest to roadway as yellow and white lines are quite thick and return multiple hough lines
    x1_max = width
    x2_max = width
    x1_min = 0
    x2_min = 0

    edges = cv2.Canny(img, 75, 150) #Edge detection
    lines = cv2.HoughLinesP(edges, 1, np.pi / 180, 25, maxLineGap=50,
                            minLineLength=50)  #Finds the lines in the image

    if lines is not None:
        for line in lines:
            x1, y1, x2, y2 = line[0]
            if colour == 'white':
                if (x1 > x1_min and x2 > x2_min) and (x1 < yellow_x and x2 < yellow_x): #If line found is closer to roadway than previous closest line and if it is on correct side of yellow line
                    m = (y2-y1)/(x2-x1) #Gradient of line
                    if m  > 0.5 or m < -0.5:    #Ignore lines with low gradient as they are probably horizontal lines that we don't want
                        x = [x1, x2]
                        y = [y1, y2]
                        final_line =[[[x1, y1, x2, y2]]]
                        white_line_prev = final_line    #Store new line as previous line for next frame
                        x1_min = x1
                        x2_min = x2
            else:
                if x1 < x1_max and x2 < x2_max:
                    m = (y2-y1)/(x2-x1)
                    if m  > 0.5 or m < -0.5:
                        x = [x1, x2]
                        y = [y1, y2]
                        final_line = [[[x1, y1, x2, y2]]]
                        yellow_line_prev = final_line
                        yellow_x = x1
                        x1_max = x1
                        x2_max = x2
            cv2.line(edges, (int(x1), int(y1)), (int(x2), int(y2)), (255, 0, 0), 2)
    if (x and y) != []: #If line has been found
        poly = np.poly1d(np.polyfit(y,x,1)) #Find equation of line

        #Find coordinates so line is drawn across entire image
        start = int(poly(max_height))
        end = int(poly(0))


        if colour == 'white':
            white_start_prev = start
            white_end_prev = end
        else:
            yellow_start_prev = start
            yellow_end_prev = end
    elif colour == 'white': #If line has not been found use line found in last good frame
        print('WARNING: PREVIOUS CALC')
        final_line = white_line_prev
        start = white_start_prev
        end = white_end_prev
    else:
        print('WARNING: PREVIOUS CALC')
        final_line = yellow_line_prev
        start = yellow_start_prev
        end = yellow_end_prev

    return final_line, start, end

#Uncomment below to run video
white_output = 'test_videos_output/cutvideo.mp4'
clip1 = VideoFileClip("test_videos/cutvideo.mp4")
white_clip = clip1.fl_image(process) #NOTE: this function expects color images!!
white_clip.write_videofile(white_output, audio=False)

#Uncomment below to run images
# test_image = cv2.imread('test_images/5fps/0001.jpg')
# t = process(test_image)
# plt.imshow(t)
# plt.show()





