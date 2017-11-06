#!/usr/bin/env python

"""
This node is used to detect what state the bike is in regards to the start/end scripts. 
If the bike is running start.sh, this node will be publishing a "1," if the bike is running
end.sh, then this node will be publishing a "0".
"""

import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import Pose2D
import time
import random 
import bikeState
import bikeSim
import mapModel
import numpy as np
from std_msgs.msg import Int32MultiArray
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import MultiArrayLayout
from std_msgs.msg import MultiArrayDimension
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
import nav
import requestHandler

def talker():
    pub = rospy.Publisher('is_bike_starting_or_stopping_pub', Int32, queue_size=10)
	rospy.init_node('state', anonymous = True)
	rate = rospy.Rate()
	while not rospy.is_shutdown():
		


if __name__=='__main__':
	try:
		talker()
	except rospy.ROSInterruptExcept:
		pass
