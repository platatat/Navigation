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

gps_data = []
kalman_state = []
kalman_state_matrix = np.matrix([0])
p_state_matrix = np.matrix([0])
count = 0
initial_lat = 0
initial_long = 0
class Kalman(object):

    def __init__(self):

        # we set initial values that would be close, but really
        # should initialize itself -- TODO

        self.gps_xy = [101,3]
        self.yaw = [1]
        self.velocity = [2]
        self.time_step = [90]
        self.ready = False

        self.pub = rospy.Publisher('kalman_pub', Float32MultiArray, queue_size=10)
        rospy.init_node('kalman')
        rospy.Subscriber("bike_state", Float32MultiArray, self.bike_state_listener)
        rospy.Subscriber("gps", Float32MultiArray, self.gps_listener)

    def bike_state_listener(self, data):
        """ROS callback for the bike_state topic"""
        yaw = data.data[9]
        #velocity = data.data[6]
        
        self.yaw = [yaw]
        
        #uncomment if getting velocity from bike_state (hall sensor)
        #self.velocity = [velocity]

    def gps_listener(self, data):
        """ROS callback for the gps topic"""
        self.ready = True
        global count,initial_lat,initial_long
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
    
        velocity = data.data[8]
        
        # uncomment if getting velocity from gps
        self.velocity = [velocity]
        
        self.time_step = [data.data[10]]
        
       
        # Converts lat long to x,y using FIXED origin
        x, y = requestHandler.math_convert(float(latitude), float(longitude))
        #Converts lat long to x,y using RELATIVE origin
        #x, y = requestHandler.math_convert_relative(float(latitude), float(longitude), float(initial_lat), float(initial_long))
        
        self.gps_xy = [x,y]
        
    def main_loop(self):

        rate = rospy.Rate(100)

        #Run until the nodes are shutdown (end.sh run OR start.sh was killed)
        while not rospy.is_shutdown():
            dim = [MultiArrayDimension('data', 1, 4)]
            layout = MultiArrayLayout(dim, 0)
            
            #wait here until GPS has been called
            while not self.ready:
                jeven = "tooter"
            
            # The Kalman filter wants the GPS data in matrix form
            #Build matrix from gps x,y coordinates, yaw, and bike velocity
            gps_matrix = np.matrix(self.gps_xy + self.yaw + self.velocity + self.time_step)
            gps_data.append(gps_matrix)
            
            # Initialize Kalman filter state
            if len(gps_data) == 1:
                P_initial = np.matrix([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
                x_pos = gps_matrix[:,0]
                y_pos = gps_matrix[:,1]
                yaw = gps_matrix[:,2]
                v = gps_matrix[:,3]
                v_0 = v.item(0)
                yaw_0 = yaw.item(0)
                x_dot_0 = v_0 * np.cos(yaw_0)
                y_dot_0 = v_0 * np.sin(yaw_0)
                s_initial = np.matrix([[x_pos.item(0)], [y_pos.item(0)], [x_dot_0], [y_dot_0]])
                output_matrix = kalman.kalman_real_time(gps_matrix, s_initial, P_initial)
                
            #Use the output of the previous call as the input to the next call
            else:
                output_matrix = kalman.kalman_real_time(gps_matrix, kalman_state_matrix, p_state_matrix)
            
            #save gps state values for later plotting
            kalman_state_matrix = output_matrix[0] 
            p_state_matrix = output_matrix[1]
            
            #Change output_matrix to a standard array for publishing
            kalman_state = output_matrix[0].flatten().tolist()[0]
            
            self.pub.publish(layout, kalman_state)
            rate.sleep()

if __name__ == '__main__':
    Kalman().main_loop()
