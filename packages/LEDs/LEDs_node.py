#!/usr/bin/env python

import os
import rospy
from duckietown import DTROS
from std_msgs.msg import String
from duckietown_msgs.msg import LEDPattern
from duckietown_msgs.srv import SetCustomLEDPattern, ChangePattern

prev_colour_list = ['switchedoff', 'switchedoff', 'switchedoff', 'switchedoff', 'switchedoff']

class LEDs(DTROS):

    def __init__(self, node_name):

        # initialize the DTROS parent class
        super(LEDs, self).__init__(node_name=node_name)

        # construct subscriber
        self.sub = rospy.Subscriber('LEDs', String, self.callback)




    def callback(self, data):
        global prev_colour_list
        freq = 10.0  #default value for frequency of blinking (in Hz)
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
        elif data.data == 'off': #All LEDs off
            colour_list = ['switchedoff', 'switchedoff', 'switchedoff', 'switchedoff', 'switchedoff']
            colour_mask = [1, 1, 1, 1, 1]
            prev_colour_list = colour_list
            print("off")
        elif data.data == 'red line' or data.data == 'yellow line': #Red line or yellow line detected in front of duckiebot (demo 3)
            colour_list = prev_colour_list
            colour_mask = [1, 1, 1, 1, 1]
            if data.data == 'red line': #Rear LEDs red if red line detected
                colour_list[1] = 'red'
                colour_list[3] = 'red'
                print("red")
            if data.data == 'yellow line': #Front LEDs yellow if yellow line detected
                colour_list[0] = 'yellow'
                colour_list[4] = 'yellow'
                print("yellow")
            prev_colour_list = colour_list
        elif data.data == '1':
            colour_list = ['switchedoff', 'red', 'switchedoff', 'swtichedoff', 'switchedoff']
            colour_mask = [1, 1, 1, 1, 1]
            print('1 Lap')
        elif data.data == '2':
            colour_list = ['switchedoff', 'red', 'red', 'swtichedoff', 'switchedoff']
            colour_mask = [1, 1, 1, 1, 1]
            print('2 Laps')
        elif data.data == '3':
            colour_list = ['switchedoff', 'red', 'red', 'red', 'switchedoff']
            colour_mask = [1, 1, 1, 1, 1]
            print('3 Laps')

            
                

        rospy.wait_for_service('/duckiebot3/led_emitter_node/set_custom_pattern')
        try:
            service = rospy.ServiceProxy('/duckiebot3/led_emitter_node/set_custom_pattern', SetCustomLEDPattern)
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
