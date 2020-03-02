#!/bin/bash

set -e

# YOUR CODE BELOW THIS LINE
# ----------------------------------------------------------------------------
#echo "This is an empty launch script. Update it to launch your application."
roscore &
sleep 5
# rosrun LEDs LEDs_node.py &
rosrun ImageStream ImageStream.py &
rosrun Demo3 task3.py
