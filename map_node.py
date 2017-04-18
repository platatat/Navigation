#!/usr/bin/env python
import rospy
import nav
from std_msgs.msg import Int32MultiArray
import numpy as np
from std_msgs.msg import MultiArrayLayout
from std_msgs.msg import MultiArrayDimension
import requestHandler
import bikeState
import mapModel

def setup_dimension():
    dim = []
    s = np.array(map_model.paths).shape[0]
    dim.append(MultiArrayDimension('paths', len(map_model.paths), s))
    dim.append(MultiArrayDimension('path', 2, 4))
    dim.append(MultiArrayDimension('point', 2, 2))
    return dim

def map_server():
    pub = rospy.Publisher('paths', Int32MultiArray, queue_size=10)
    rospy.init_node('map', anonymous=True)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        dim = setup_dimension()
        layout = MultiArrayLayout(dim, 0)
        rospy.loginfo(map_model.paths)
        pub.publish(layout, list(np.array(map_model.paths).ravel()))
        rate.sleep()

if __name__ == "__main__":
    new_bike = bikeState.Bike(-5, -5, 0.1, 0, 0, 0, 3.57)
    waypoints = requestHandler.parse_json()
    map_model = mapModel.Map_Model(new_bike, waypoints, [], [])
    map_server()