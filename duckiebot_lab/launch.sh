#!/bin/bash

set -e

# YOUR CODE BELOW THIS LINE
# ----------------------------------------------------------------------------
#echo "This is an empty launch script. Update it to launch your application."
#roscore &
#sleep 5
#rosrun ImageStream ImageStream.py &
rosparam set /duckiebot3/camera_node/framerate 30 &
rosparam set /duckiebot3/kinematics_node/omega_max 45 &
rosrun LEDs LEDs_node.py &
rosrun demo4 line_detect_improved.py &
#rosrun demo4 trial.py
sleep 1
rosrun demo5 LapCounting.py &
rosrun demo5 StopSign.py
# rosrun recording recordFile.py
