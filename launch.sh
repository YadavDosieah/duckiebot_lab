#!/bin/bash

set -e

# YOUR CODE BELOW THIS LINE
# ----------------------------------------------------------------------------
#echo "This is an empty launch script. Update it to launch your application."
roscore &
sleep 5
rosrun ImageStream ImageStream.py &
rosrun Demo3 task3.py &
rosrun LEDs LEDs_node.py
