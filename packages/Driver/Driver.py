#!/usr/bin/env python

import os
import rospy
from duckietown import DTROS
from std_msgs.msg import String
from duckietown_msgs.msg import WheelsCmdStamped, BoolStamped
from time import sleep

class Driver(DTROS):

    def __init__(self, node_name):

        # initialize the DTROS parent class
        super(Driver, self).__init__(node_name=node_name)

        # construct publisher
        self.pub = rospy.Publisher('/duckiebotX/wheels_driver_node/wheels_cmd', WheelsCmdStamped, queue_size=1)
        self.pub2 = rospy.Publisher('LEDs', String, queue_size=1)

        # construct subscriber
        self.sub = rospy.Subscriber('Driver', String, self.callback)

        self.Speed = 0.5 #Speed at which duckiebot moves
        self.ForwardTime = 5.0 #Time for which duckiebot moves forward
        self.TurnTime = 2.0 #Time for which duckiebot rotates


    def run(self):
        self.forward()
        self.rotate()
        self.forward()
        self.stop()


    def callback(self, data):
        if data.data == 'demo': #Command to run full demo
            self.run()
        elif data.data.startswith('forward'): #Command to move forward by x seconds (e.g forward5)
            DataList = data.data.split('forward')
            self.ForwardTime = float(DataList[1])
            self.forward()
            self.stop()
        elif data.data.startswith('rotate'):#Command to rotate by x seconds(e.g rotate2)
            DataList = data.data.split('rotate')
            self.TurnTime = float(DataList[1])
            self.rotate()
            self.stop()
        elif data.data.startswith('speed'):#Command to set speed (e.g speed0.5)
            DataList = data.data.split('speed')
            self.Speed = float(DataList[1])
            if(self.Speed >= 0 and self.Speed <= 1):
                print('Setting speed to:{}'.format(self.Speed))
            else:
                print('Speed must be within 0 and 1')



    def forward(self):
        #Move Forward for x seconds (defined by ForwardTime)
        msg = WheelsCmdStamped()
        msg.header.stamp = rospy.get_rostime()
        msg.vel_left = self.Speed
        msg.vel_right = self.Speed
        self.pub.publish(msg)
        self.pub2.publish('straight')
        print('Moving forward for {}s'.format(self.ForwardTime))
        sleep(self.ForwardTime)


    def rotate(self):
        #Rotate for x seconds (defined by TurnTime)
        msg = WheelsCmdStamped()
        msg.header.stamp = rospy.get_rostime()
        msg.vel_left = -self.Speed
        msg.vel_right = self.Speed
        self.pub.publish(msg)
        self.pub2.publish('left')
        print('Rotating for {}s'.format(self.TurnTime))
        sleep(self.TurnTime)


    def stop(self):
        #Stop
        msg = WheelsCmdStamped()
        msg.header.stamp = rospy.get_rostime()
        msg.vel_left = 0
        msg.vel_right = 0
        self.pub.publish(msg)
        self.pub2.publish('end')
        print('Stopping')

if __name__ == '__main__':
    # create the node
    node = Driver(node_name='Driver')
    # run node
    node.run()
    # keep spinning
    rospy.spin()
