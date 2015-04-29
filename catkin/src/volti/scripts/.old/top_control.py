#!/usr/bin/env python

import roslib; roslib.load_manifest('volti')
import rospy
import math
from   time         import time
from   volti.msg    import float32_3
from   volti.msg    import float32_12
from   numpy        import *
from   numpy.linalg import inv 

grad2rad = 3.141592/180.0

R_IDEM = array([[1, 0, 0],[0, 1, 0],[0, 0, 1]])
R_roll_180 = array([[1, 0, 0],[0, cos(3.1416), -sin(3.1416)],[0, sin(3.1416), cos(3.1416)]])
R_pitch_180 = array([[cos(3.1416), 0, sin(3.1416)],[0, 1, 0],[-sin(3.1416), 0, cos(3.1416)]])

rospy.init_node("top_control_node")

def processIMU1_message(rawMsg):

    roll = rawMsg.data[0]
    pitch = rawMsg.data[1]

def processIMU2_message(rawMsg):

    roll2 = rawMsg.data[0]
    pitch2 = rawMsg.data[1]

sub1 = rospy.Subscriber('imu_plate_3', float32_3, processIMU1_message)
sub2 = rospy.Subscriber('imu_pendu_3', float32_3, processIMU2_message)