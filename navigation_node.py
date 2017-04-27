#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import Pose2D
from geometry_msgs.msg import Twist
import nav
import numpy as np
from std_msgs.msg import Int32MultiArray
from std_msgs.msg import Float32MultiArray
import bikeState
import mapModel
import requestHandler


def callback(data):
    new_nav.map_model.bike.xy_coord = (data.x, data.y)
    new_nav.map_model.bike.direction = data.theta

def path_parse(data):
    d = np.array(data.data).reshape(len(data.data)/4, 2, 2)
    new_map.paths = d

def update_bike_state(data):
    d = data.data
    #new_bike.xB = d[0]
    #new_bike.yB = d[1]
    new_bike.phi = d[5]
    new_bike.delta = d[2]
    new_bike.w_r = d[4]
    # new_bike.v = d[] #gps or 6
    # new_bike.turning_r = d[7] LOOKEDUP


def update_bike_xy(data):
    lat = data.data[0]
    lon = data.data[1]
    psi = data.data[7]
    velocity = data.data[8]

    xy_point = requestHandler.math_convert(lat, lon)

    new_bike.psi = psi
    new_bike.v = velocity
    new_bike.xB = xy_point[0]
    new_bike.yB = xy_point[1]

def keyboard_update(data):
    #rospy.loginfo(data) 
    x = data.linear.x
    z = data.angular.z
    v = .5
    if x>0 and z == 0: #up
        new_bike.yB += v 
    if x<0 and z == 0: #down
        new_bike.yB -= v 
    if z>0 and x == 0: #left
        new_bike.xB -=v        
    if z<0 and x == 0: #right
        new_bike.xB +=v
    if x>0 and z > 0: #ccw
        psi = new_bike.psi
        new_bike.psi = (psi + np.pi/12)%(2*np.pi) 
    if x>0 and z < 0: #cw
        psi = new_bike.psi
        new_bike.psi = (psi + np.pi/12)%(2*np.pi)

def talker():
    pub = rospy.Publisher('nav_instr', Int32, queue_size=10)
    rospy.init_node('navigation', anonymous=True)
    # Subscribe to topic "bike_state" to get data and then call update_bike_state with data
    rospy.Subscriber("cmd_vel", Twist, keyboard_update)
    rospy.Subscriber("bike_state", Float32MultiArray, update_bike_state) 
    rospy.Subscriber("gps", Float32MultiArray, update_bike_xy)
    rospy.Subscriber("paths", Int32MultiArray, path_parse) 
    rate = rospy.Rate(100)
    while not rospy.is_shutdown():
        new_map = new_nav.map_model
        rospy.loginfo((new_bike.xB, new_bike.yB, new_bike.psi, new_nav.direction_to_turn()))
        pub.publish(new_nav.direction_to_turn())
        rate.sleep()

if __name__ == '__main__':
    try:
        new_bike = bikeState.Bike(0, -10, 0.1, np.pi/3, 0, 0, 3.57)
        waypoints = [(0.1, 0.1), (30.1, 0.1), (31.1, 0.1)]
        new_map = mapModel.Map_Model(new_bike, waypoints, [], [])
        new_nav = nav.Nav(new_map)
        talker()
    except rospy.ROSInterruptException:
        rospy.loginfo('here')
        pass
