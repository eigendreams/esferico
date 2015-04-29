#!/usr/bin/env python

import roslib; roslib.load_manifest('volti')
import rospy
import math

from volti.msg import float32_3
from sensor_msgs.msg import Imu
from tf.transformations import euler_from_quaternion

rospy.init_node("imu_to_imu_3")

pub_imu_plate_proc_3 = rospy.Publisher('imu_plate_proc_3', float32_3, queue_size = 1)
pub_imu_pendu_proc_3 = rospy.Publisher('imu_pendu_proc_3', float32_3, queue_size = 1)
pub_imu_plate_3      = rospy.Publisher('imu_plate_3',      float32_3, queue_size = 1)
pub_imu_pendu_3      = rospy.Publisher('imu_pendu_3',      float32_3, queue_size = 1)

imu_plate_proc_3_msg = float32_3()
imu_pendu_proc_3_msg = float32_3()
imu_plate_3_msg      = float32_3()
imu_pendu_3_msg      = float32_3()

def imu_plate_proc_cb(rawMsg):
    #
    e_plate_proc = euler_from_quaternion([rawMsg.orientation.x, rawMsg.orientation.y, rawMsg.orientation.z, rawMsg.orientation.w])
    #
    imu_plate_proc_3_msg.data[0] = e_plate_proc[0]
    imu_plate_proc_3_msg.data[1] = e_plate_proc[1]
    imu_plate_proc_3_msg.data[2] = e_plate_proc[2]
    #
    pub_imu_plate_proc_3.publish(imu_plate_proc_3_msg)
    #
def imu_pendu_proc_cb(rawMsg):
    #
    e_pendu_proc = euler_from_quaternion([rawMsg.orientation.x, rawMsg.orientation.y, rawMsg.orientation.z, rawMsg.orientation.w])
    #
    imu_pendu_proc_3_msg.data[0] = e_pendu_proc[0]
    imu_pendu_proc_3_msg.data[1] = e_pendu_proc[1]
    imu_pendu_proc_3_msg.data[2] = e_pendu_proc[2]
    #
    pub_imu_pendu_proc_3.publish(imu_pendu_proc_3_msg)
    #
def imu_plate_cb(rawMsg):
    #
    e_plate = euler_from_quaternion([rawMsg.orientation.x, rawMsg.orientation.y, rawMsg.orientation.z, rawMsg.orientation.w])
    #
    imu_plate_3_msg.data[0] = e_plate[0]
    imu_plate_3_msg.data[1] = e_plate[1]
    imu_plate_3_msg.data[2] = e_plate[2]
    #
    pub_imu_plate_3.publish(imu_plate_3_msg)
    #
def imu_pendu_cb(rawMsg):
    #
    e_pendu = euler_from_quaternion([rawMsg.orientation.x, rawMsg.orientation.y, rawMsg.orientation.z, rawMsg.orientation.w])
    #
    imu_pendu_3_msg.data[0] = e_pendu[0]
    imu_pendu_3_msg.data[1] = e_pendu[1]
    imu_pendu_3_msg.data[2] = e_pendu[2]
    #
    pub_imu_pendu_3.publish(imu_pendu_3_msg)
    #
    
sub_imu_plate_proc = rospy.Subscriber("imu_plate_proc", Imu, imu_plate_proc_cb)
sub_imu_pendu_proc = rospy.Subscriber("imu_pendu_proc", Imu, imu_pendu_proc_cb)
sub_imu_plate      = rospy.Subscriber("imu_plate",      Imu, imu_plate_cb)
sub_imu_pendu      = rospy.Subscriber("imu_pendu",      Imu, imu_pendu_cb)

rospy.spin()