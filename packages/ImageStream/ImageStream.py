#!/usr/bin/env python

import os
import rospy
from duckietown import DTROS
from std_msgs.msg import String
from sensor_msgs.msg import CompressedImage
import cv2
import numpy as np
from cv_bridge import CvBridge

<<<<<<< HEAD
=======
global frame
>>>>>>> b1fd86f5f0b695b457deeb3fb978fc793d6c1bfc
frame = None

class ImageStream(DTROS):

    def __init__(self, node_name):

        # initialize the DTROS parent class
        super(ImageStream, self).__init__(node_name=node_name)

        # construct publisher
        self.pub = rospy.Publisher('/duckiebot3/camera_node/image/compressed', CompressedImage, queue_size=1)
        self.cam =cv2.VideoCapture(-1)
        bridge = CvBridge()
        rospy.sleep(2)
        self.frame = None

    def run(self):
        global frame
        if not self.cam.isOpened():
            print("Webcam is not available")
            return -1

        ret, frame = self.cam.read()
        # image_message = bridge.cv2_to_imgmsg(frame, encoding="bgr8")

        #### Create CompressedIamge ####
        msg = CompressedImage()
        msg.header.stamp = rospy.Time.now()
        msg.format = "jpeg"
        msg.data = np.array(cv2.imencode('.jpg', frame)[1]).tostring()
        # Publish new image
        self.pub.publish(msg)
        #rospy.sleep(1)


if __name__ == '__main__':
    global frame
    # create the node
    node = ImageStream(node_name='ImageStream')
    while not rospy.is_shutdown():
        # run node
        node.run()
        cv2.imshow('frame',frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            cv2.destroyAllWindows()
    # keep spinning
    rospy.spin()
