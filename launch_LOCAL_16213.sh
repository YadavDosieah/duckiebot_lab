#!/bin/bash

set -e

# YOUR CODE BELOW THIS LINE
# ----------------------------------------------------------------------------
#echo "This is an empty launch script. Update it to launch your application."
#roscore &
#sleep 5
#rosrun ImageStream ImageStream.py &
rosparam set /duckiebot3/camera_node/framerate 5
# rosrun Demo3 demo3.py &
rosrun demo4 lane_detection.py
# rosrun LEDs LEDs_node.py
