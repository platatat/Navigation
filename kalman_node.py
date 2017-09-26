#!/usr/bin/env python
"""
Used for ROS communication. It allows real-time 'filtering' of position data
on the bike by subscribing to data from different sensors (GPS + IMU). When
testing, use "bash start.sh run_with_kalman" to run this node.
"""
import csv
import numpy as np
import gps_assisted_simulator_node
import subprocess
from std_msgs.msg import Float32
import rospy
import kalman
import geometry
import requestHandler
from std_msgs.msg import Int32MultiArray
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import MultiArrayLayout
from std_msgs.msg import MultiArrayDimension

class Kalman(object):

    def __init__(self):

        # we set initial values that would be close, but really
        # should initialize itself -- TODO

        self.gps_xy = [101,3]
        self.bike_yv = [1,2]
        self.time_step = [90]

        #Data from bike_state and gps respectively
        #state that is published to ROS
        self.kalman_state = []
        #State used for kalman_real_time
        self.kalman_state_matrix = np.matrix([0])
        self.p_state_matrix = np.matrix([0])

        self.pub = rospy.Publisher('kalman_pub', Float32MultiArray, queue_size=10)
        rospy.init_node('kalman')
        rospy.Subscriber("bike_state", Float32MultiArray, self.bike_state_listener)
        rospy.Subscriber("gps", Float32MultiArray, self.gps_listener)

    def bike_state_listener(self, data):
        """ROS callback for the bike_state topic"""
        velocity = data.data[6]
        yaw = data.data[9]
        self.bike_yv = [yaw, velocity]

    def gps_listener(self, data):
        """ROS callback for the gps topic"""
        #Important fields from data
        latitude = data.data[0] # In degrees
        longitude = data.data[1]
        self.time_step = [data.data[10]]
        #psi = data.data[7] # psi = heading in radians
        #velocity = data.data[8]
        # Converts lat long to x,y
        x, y = requestHandler.math_convert(float(latitude), float(longitude))

        # Outlier detection - if dist from last x,y is more than 50 m, ignore
       # if geometry.distance([x,y], self.gps_xy) > 50:
        #    return

        # gps current state - only relevant fields
        self.gps_xy = [x, y]

        # If the GPS data is nonzero, assume that the GPS is ready
        if not rospy.get_param("/gps_ready", False) and longitude != 0.0 and latitude != 0.0:
            rospy.set_param("/gps_ready", True)

    def main_loop(self):

        # Wait until the GPS is ready
        rate = rospy.Rate(20)
        while not rospy.get_param("/gps_ready", False):
            rate.sleep()

        # Initialize Kalman filter state
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

        rate = rospy.Rate(100)

        #Run until the nodes are shutdown (end.sh run OR start.sh was killed)
        while not rospy.is_shutdown():
            dim = [MultiArrayDimension('data', 1, 4)]
            layout = MultiArrayLayout(dim, 0)

            # The Kalman filter wants the GPS data in matrix form
            #Build matrix from gps x,y coordinates and bike velocity and yaw
            gps_matrix = np.matrix(self.gps_xy + self.bike_yv + self.time_step)

            #save gps state values for later plotting
            self.kalman_state_matrix, self.p_state_matrix = kalman.kalman_real_time(
                    gps_matrix, self.kalman_state_matrix, self.p_state_matrix)

            #Change output_matrix to a standard array for publishing
            self.kalman_state = self.kalman_state_matrix.flatten().tolist()[0]

            self.pub.publish(layout, self.kalman_state)
            rate.sleep()

if __name__ == '__main__':
    Kalman().main_loop()
