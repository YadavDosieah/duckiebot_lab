#!/usr/bin/env python

import os
import rospy
from duckietown import DTROS
from std_msgs.msg import String
from duckietown_msgs.msg import LEDPattern
from duckietown_msgs.srv import SetCustomLEDPattern, ChangePattern

class LEDs(DTROS):

    def __init__(self, node_name):

        # initialize the DTROS parent class
        super(LEDs, self).__init__(node_name=node_name)

        # construct subscriber
        self.sub = rospy.Subscriber('LEDs', String, self.callback)




    def callback(self, data):
        freq = 2.0  #default value for frequency of blinking (in Hz)
        freq_mask = [0,0,0,0,0] #default value for frequency mask

        #Changes LEDs depending on message from publisher
        if data.data == 'straight':  #White LEDs on front when travelling straight
            colour_list = ['white', 'switchedoff', 'switchedoff', 'switchedoff', 'white']
            colour_mask = [1, 1, 1, 1, 1]
            print('straight')
        elif data.data == 'right':  #Blinking yellow when turning right
            colour_list = ['white', 'switchedoff', 'switchedoff', 'switchedoff', 'yellow']
            colour_mask = [1, 1, 1, 1, 1]
            freq_mask = [0, 0, 0, 0, 1] #Only front right light blinks
            print('right')
        elif data.data == 'left':    #Blinking yellow when turning left
            colour_list = ['yellow', 'switchedoff', 'switchedoff', 'switchedoff', 'white']
            colour_mask = [1, 1, 1, 1, 1]
            freq_mask = [1, 0, 0, 0, 0] #Only front left light blinks
            print('left')
        elif data.data == 'end': #Red LEDs on rear when at end
            colour_list = ['white', 'red', 'switchedoff', 'red', 'white']
            colour_mask = [1, 1, 1, 1, 1]
            print('end')

        rospy.wait_for_service('/duckiebotX/led_emitter_node/set_custom_pattern')
        try:
            service = rospy.ServiceProxy('/duckiebotX/led_emitter_node/set_custom_pattern', SetCustomLEDPattern)
            msg = LEDPattern()
            msg.color_list = colour_list
            msg.color_mask = colour_mask
            msg.frequency = freq
            msg.frequency_mask = freq_mask
            response = service(msg)
            rospy.loginfo(response)
        except rospy.ServiceException, e:
            print("Service call failed: %s"%e)





if __name__ == '__main__':
    # create the node
    node = LEDs(node_name='LEDs')
    # keep spinning
    rospy.spin()
