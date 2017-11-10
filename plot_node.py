#!/usr/bin/env python
"""
Used for ROS communication. It allows real-time 'filtering' of position data
on the bike by subscribing to data from different sensors (GPS + IMU). When
testing, use "bash start.sh run_with_kalman" to run this node.
"""

import numpy as np
from std_msgs.msg import Float32
import rospy
import kalman
import geometry
import requestHandler
from std_msgs.msg import Int32MultiArray
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import MultiArrayLayout
from std_msgs.msg import MultiArrayDimension
from matplotlib import pyplot as plt

class Plotter(object):

    def __init__(self):
        rospy.init_node("plotter")
        rospy.Subscriber('kalman_pub', Float32MultiArray, self.kalman_listener)
        rospy.Subscriber("gps", Float32MultiArray, self.gps_listener)

    def kalman_listener(self, data):
        """ROS callback for the kalman_pub topic"""
        x = data.data[0]
        y = data.data[1]
        plt.scatter(x,y, c='b')
        plt.draw()
        plt.pause(0.00000000001)
        

    def gps_listener(self, data):
        """ROS callback for the gps topic"""
        #print(self.ready)
        #Check if this is our first GPS reading --
        # not robust but will work for now if we start test at the right spot
        if count == 0:
            #Re-defines the origins 
            initial_lat = data.data[0]
            initial_long = data.data[1]
            count = 1
        #Important fields from data
        latitude = data.data[0] # In degrees
        longitude = data.data[1]
        yaw = data.data[7]
        velocity = data.data[8]

        # Converts lat long to x,y using FIXED origin
        #x, y = requestHandler.math_convert(float(latitude), float(longitude))
        #Converts lat long to x,y using RELATIVE origin
        x, y = requestHandler.math_convert_relative(float(latitude), float(longitude), float(initial_lat), float(initial_long))
        plt.scatter(x,y, c='r')
        plt.draw()
        plt.pause(0.00000000001)


if __name__ == '__main__':
    plt.ion()
    plt.show()
    rospy.spin()
