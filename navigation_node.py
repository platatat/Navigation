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
    
    #Uncomment for yaw from bike_state (IMU)
    #new_bike.psi = d[9] #heading/yaw (IMU)
    
    #Uncomment for velocity from bike_state (hall sensors)
    #new_bike.v = d[6] # velocity (hall sensors)

#callback from kalman_pub -- data.data = [x,y,x_dot,y_dot]
def update_xy(data):
    """Updates the bike object with data from kalman_pub"""
    
    #x and y from kalman
    new_bike.xB = data.data[0] #x position (kalman)
    new_bike.yB = data.data[1] #y position (kalman)

#callback from gps 
def update_gps(data):
    """Updates the bike object with data from gps"""
    
    #Uncomment for yaw from the GPS
    new_bike.psi = np.deg2rad(data.data[7]) #yaw/heading (GPS)
    
    #Uncomment for velocity from the GPS
    new_bike.v = data.data[8] #velocity (GPS)
    
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
        print("Map_model bike x y: {}, {}".format(new_nav.map_model.bike.xB, new_nav.map_model.bike.yB))
        #rospy.loginfo((new_bike.xB, new_bike.yB, new_bike.psi, new_nav.direction_to_turn()))
        pub.publish(new_nav.get_steering_angle())
        rate.sleep()

if __name__ == '__main__':
    try:
        old_psi = 0
        old_v = 0
        d_psi = 0
        new_bike = bikeState.Bike(0, -10, 0.1, np.pi/3, 0, 0, 3.57)
        
        #straight path bottom of stairs to middle intersection
        waypoints = [(133.74892645377858, -432.5469806370678), (114.05523219700194, -395.85300512410294)]
        
        #long path
        #waypoints = [(164.92918389320673, -412.53122538008699), (160.82636519097008, -408.08352424480717), (158.36456020192119, -400.29993577850547), (157.54391784874556, -395.85215731941992), (152.62045116162616, -385.84472352909091), (141.95309049412472, -369.16571007809335), (133.74760279568892, -363.6061261255179), (124.72163122543957, -360.27044577941609), (118.1572971243023, -358.04666168562972), (110.7724130084174, -354.71093523541589), (97.643830726023069, -354.7111316365864), (98.464555727252943, -368.05451128333181), (101.74674006642893, -370.2783626484474), (102.56727829387685, -370.27835061499496), (103.3878330236748, -371.39028775136137), (114.05523219700194, -395.85300512410294), (134.56949349422308, -433.6589141004269)]
        
        new_map = mapModel.Map_Model(new_bike, waypoints, [], [])
        new_nav = nav.Nav(new_map)
        talker()
    except rospy.ROSInterruptException:
        rospy.loginfo('here')
        pass
    
    
    
    
    
