#!/usr/bin/env python
"""Used for ROS communication. Communicates between various sensors to update bike object
and allows navigation algorithm to run on the bike."""

import rospy
from std_msgs.msg import Int32
from std_msgs.msg import Float32
from geometry_msgs.msg import Pose2D
from geometry_msgs.msg import Twist
import nav
import numpy as np
from std_msgs.msg import Int32MultiArray
from std_msgs.msg import Float32MultiArray
import bikeState
import mapModel
import requestHandler

#def callback(data):
 #   new_nav.map_model.bike.xy_coord = (data.x, data.y)
 #  new_nav.map_model.bike.direction = data.theta

#Callback for paths
def path_parse(data):
    d = np.array(data.data).reshape(len(data.data)/4, 2, 2)
    new_map.paths = d


#callback from bike_state
def update_bike_state(data):
    """Updates the bike object with data from bike_state"""
    d = data.data

    new_bike.phi = d[5] #lean angle/roll (IMU)
    new_bike.delta = d[2] #steer angle (encoder)
    new_bike.w_r = d[4] #lean rate (IMU)
    new_bike.psi = d[9] #heading/yaw (IMU)
    
    #Uncomment for velocity from bike_state (hall sensors)
    new_bike.v = d[6] # velocity (hall sensors)

#callback from kalman_pub -- data.data = [x,y,x_dot,y_dot]
def update_xy(data):
    """Updates the bike object with data from kalman_pub"""
    
    #x and y from kalman
    new_bike.xB = data.data[0] #x position (kalman)
    new_bike.yB = data.data[1] #y position (kalman)

#callback from gps 
def update_gps(data):
    """Updates the bike object with data from gps"""
    
    #Comment for velocity from the GPS
    pass
    
    #Uncomment for velocity from the GPS
    # new_bike.v = data.data[8] #velocity (GPS)
    
    #global old_gps_set
    #curr_gps_set = (data.data[0], data.data[1], data.data[7], data.data[8])
    #if curr_gps_set == old_gps_set:
    #    rospy.loginfo("Rejecting data from gps")
    #    return
    #old_gps_set = curr_gps_set

#    if data.data[0] == 0 and data.data[1] == 0:
#       not_ready = True
#    else:
    #if (True):
        # not_ready = False
        # latitude = data.data[0] # In degrees
        # longitude = data.data[1]
        # psi = data.data[7] # psi = heading in radians

    #if data.data[0] == 0 and data.data[1] == 0:
     #   not_ready = True
    #else:
        #time_since_last = data.data[5]
        #EPSILON = 25
        #if time_since_last > EPSILON:
            #not_ready = False
            #lat = data.data[0] # In degrees 
            #lon = data.data[1]
        #     #psi = data.data[7] # This needs to be in rads
        #     #velocity = data.data[8]
        # xy_point = requestHandler.math_convert(latitude, longitude)
        # 
        # 
        #     #d_psi = float(psi - old_psi)/old_time_since_last
        # new_bike.psi = psi
        #     #Update old velocity and angle for extrapolation
        #     #old_v = velocity
        #     #old_psi = psi
        # new_bike.xB = xy_point[0]
        # new_bike.yB = xy_point[1]
        #else:
         #   new_psi = d_psi*time_since_last + old_psi
          #  new_x = velocity*cos(new_psi)
           # new_y = velocity*sin(new_psi)
        #old_time_since_last = data.data[5]

def talker():
    pub = rospy.Publisher('nav_instr', Float32, queue_size=10)
    rospy.init_node('navigation', anonymous=True)

    rospy.Subscriber("bike_state", Float32MultiArray, update_bike_state) 
    rospy.Subscriber("gps", Float32MultiArray, update_gps)
    rospy.Subscriber("kalman_pub", Float32MultiArray, update_xy)
    rospy.Subscriber("paths", Float32MultiArray, path_parse)
    
    rate = rospy.Rate(100)
    while not rospy.is_shutdown():
        new_map = new_nav.map_model
        #rospy.loginfo((new_bike.xB, new_bike.yB, new_bike.psi, new_nav.direction_to_turn()))
        pub.publish(new_nav.get_steering_angle())
        rate.sleep()

if __name__ == '__main__':
    try:
        old_psi = 0
        old_v = 0
        d_psi = 0
        new_bike = bikeState.Bike(0, -10, 0.1, np.pi/3, 0, 0, 3.57)
        # waypoints = requestHandler.parse_json(True)
        waypoints = [(0.1, 0.1), (30.1, 0.1), (31.1, 0.1)]
        new_map = mapModel.Map_Model(new_bike, waypoints, [], [])
        new_nav = nav.Nav(new_map)
        talker()
    except rospy.ROSInterruptException:
        rospy.loginfo('here')
        pass
