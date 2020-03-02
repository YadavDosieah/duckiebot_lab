#!/bin/bash

set -e

# YOUR CODE BELOW THIS LINE
# ----------------------------------------------------------------------------
#echo "This is an empty launch script. Update it to launch your application."
<<<<<<< HEAD
#roscore &
#sleep 5
#rosrun ImageStream ImageStream.py &
rosparam set /duckiebot3/camera_node/framerate 5
=======
roscore &
sleep 5
<<<<<<< HEAD
# rosrun LEDs LEDs_node.py &
rosrun ImageStream ImageStream.py &
rosrun Demo3 task3.py
=======
rosrun ImageStream ImageStream.py &
>>>>>>> ef3f6c1aa582383cbd97995fe16558e7a213601e
# rosrun Demo3 demo3.py &
rosrun demo4 lane_detection.py
# rosrun LEDs LEDs_node.py
>>>>>>> b1fd86f5f0b695b457deeb3fb978fc793d6c1bfc
