#!/usr/bin/env python

import roslib; roslib.load_manifest('volti')
import rospy
import math
from   volti.msg import float32_12

from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3Stamped
from tf.transformations import euler_from_quaternion

degrees2rad = math.pi/180.0
rospy.init_node("razor_node_iface")
pub = rospy.Publisher('imu/raw', Imu, queue_size=10)
pubmag = rospy.Publisher('imu/mag', Vector3Stamped, queue_size=10)
pubimufiltered = rospy.Publisher('i1_test', float32_12, queue_size=10)

imuMsg = Imu()
imumagMsg = Vector3Stamped()
imufilteredMsg = float32_12()

imuMsg.orientation_covariance = [
0.0025 , 0 , 0,
0, 0.0025, 0,
0, 0, 0.0025
]

imuMsg.angular_velocity_covariance = [
0.02, 0 , 0,
0 , 0.02, 0,
0 , 0 , 0.02
]

imuMsg.linear_acceleration_covariance = [
0.04 , 0 , 0,
0 , 0.04, 0,
0 , 0 , 0.04
]

accel_factor = 9.806 / 256.0    # sensor reports accel as 256.0 = 1G (9.8m/s^2). Convert to m/s^2.

def imu1_process(rawMsg):
    #
    imuMsg.linear_acceleration.x = float(rawMsg.data[3]) * accel_factor
    imuMsg.linear_acceleration.y = float(rawMsg.data[4]) * accel_factor
    imuMsg.linear_acceleration.z = float(rawMsg.data[5]) * accel_factor
    #
    imuMsg.angular_velocity.x = float(rawMsg.data[9]) * 0.01745329252 * 0.06957
    imuMsg.angular_velocity.y = float(rawMsg.data[10]) * 0.01745329252 * 0.06957
    imuMsg.angular_velocity.z = float(rawMsg.data[11]) * 0.01745329252 * 0.06957
    #
    imuMsg.header.stamp = rospy.Time.now()
    imuMsg.header.frame_id = 'base'
    pub.publish(imuMsg)
    #
    imumagMsg.vector.x = float(rawMsg.data[6])
    imumagMsg.vector.y = float(rawMsg.data[7])
    imumagMsg.vector.z = float(rawMsg.data[8])
    imumagMsg.header.stamp = rospy.Time.now()
    imumagMsg.header.frame_id = 'base'
    pubmag.publish(imumagMsg)
    #
def imufiltercb(message):
    #
    e = euler_from_quaternion([message.orientation.x, message.orientation.y, message.orientation.z, message.orientation.w])
    #
    imufilteredMsg.data[0] = e[0]
    imufilteredMsg.data[1] = e[1]
    imufilteredMsg.data[2] = e[2]
    #
    pubimufiltered.publish(imufilteredMsg)
    #

sub = rospy.Subscriber("i1", float32_12, imu1_process)
subfilter = rospy.Subscriber('imu/data', Imu, imufiltercb)

rospy.spin()