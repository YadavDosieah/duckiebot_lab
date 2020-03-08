#!/usr/bin/env python

import os
import rospy

from duckietown import DTROS
from std_msgs.msg import String
from duckietown_msgs.msg import WheelsCmdStamped, Twist2DStamped

class KinematicsNode_Trim(DTROS):

    def __init__(self, node_name):
        # Initialize the DTROS parent class
        super(KinematicsNode_Trim, self).__init__(node_name=node_name)

        self.pub = self.publisher("velocity_angle", Twist2DStamped, queue_size=1)
        self.sub = self.subscriber("kin_trim", String, self.callback)

    def callback(self, data):
        if data.data == 'forward':
            self.forward()
        elif data.data == 'stop':
            self.stop()
        else:
            self.turn(data)

    def forward(self):
        msg = Twist2DStamped()
        msg.header.stamp = rospy.get_rostime()
        msg.v = 0.5
        msg.omega = 0
        self.pub.publish(msg)

    def turn(self, data):
        msg = Twist2DStamped()
        msg.header.stamp = rospy.get_rostime()
        msg.v = 0.5
        msg.omega = float(data)
        self.pub.publish(msg)

    def stop(self):
        msg = Twist2DStamped()
        msg.header.stamp = rospy.get_rostime()
        msg.v = 0
        msg.omega = 0
        self.pub.publish(msg)

if __name__ == '__main__':
    # create the node
    node = KinematicsNode_Trim(node_name='KinematicsNode_Trim')
    # run node
    node.run()
    # keep spinning
    rospy.spin()



