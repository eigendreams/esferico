#!/usr/bin/env python

# Copyright (c) 2012, Tang Tiong Yew
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#    * Neither the name of the Willow Garage, Inc. nor the names of its
#      contributors may be used to endorse or promote products derived from
#       this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import rospy
from   visual       import *
import math
from   time         import time
from   volti.msg    import float32_3
from   volti.msg    import float32_12
from   numpy        import *
from   numpy.linalg import inv 

grad2rad = 3.141592/180.0

# Main scene
scene=display(title="IMU rotation matrix test")
scene.range=(1.2,1.2,1.2)
scene.forward = (1,0,-0.25)
scene.up=(0,0,1)
scene.width=500
scene.height=500
scene.y=200

# Main scene objects
scene.select()
# Reference axis (x,y,z)
arrow(color=color.green,axis=(1,0,0), shaftwidth=0.02, fixedwidth=1)
arrow(color=color.green,axis=(0,1,0), shaftwidth=0.02 , fixedwidth=1)
arrow(color=color.green,axis=(0,0,1), shaftwidth=0.02, fixedwidth=1)
# labels
label(pos=(0,0,0.8),text="Ground",box=0,opacity=0)
label(pos=(1,0,0),text="X",box=0,opacity=0)
label(pos=(0,-1,0),text="Y",box=0,opacity=0)
label(pos=(0,0,-1),text="Z",box=0,opacity=0)
# IMU object
x_test = arrow(color=color.blue,axis=(1,0,0), shaftwidth=0.02, fixedwidth=1)
y_test = arrow(color=color.blue,axis=(0,-1,0), shaftwidth=0.02 , fixedwidth=1)
z_test = arrow(color=color.blue,axis=(0,0,-1), shaftwidth=0.02, fixedwidth=1)

x_test_2 = arrow(color=color.red,axis=(1,0,0), shaftwidth=0.02, fixedwidth=1)
y_test_2 = arrow(color=color.red,axis=(0,-1,0), shaftwidth=0.02 , fixedwidth=1)
z_test_2 = arrow(color=color.red,axis=(0,0,-1), shaftwidth=0.02, fixedwidth=1)

R_IDEM = array([[1, 0, 0],[0, 1, 0],[0, 0, 1]])
R_roll_180 = array([[1, 0, 0],[0, cos(3.1416), -sin(3.1416)],[0, sin(3.1416), cos(3.1416)]])
R_pitch_180 = array([[cos(3.1416), 0, sin(3.1416)],[0, 1, 0],[-sin(3.1416), 0, cos(3.1416)]])

rospy.init_node("imu_3d_dcm_nyaw_node")

def angleRange(data):
    #
    if (data > 0):
        return data - 3.1416
    if (data < 0):
        return data + 3.1416
    return data
    #

def processIMU1_message(rawMsg):

    roll = angleRange(rawMsg.data[0])
    pitch = rawMsg.data[1]
    yaw = rawMsg.data[2] * 0

    R_yaw_z = array([[cos(yaw), sin(yaw), 0], [-sin(yaw), cos(yaw), 0], [0, 0, 1]])
    R_pitch_y = array([[cos(pitch), 0, sin(pitch)],[0, 1, 0],[-sin(pitch), 0, cos(pitch)]])
    R_roll_x = array([[1, 0, 0],[0, cos(roll), -sin(roll)],[0, sin(roll), cos(roll)]])
   
    x_axis_imu = (dot(R_yaw_z, dot(R_pitch_y, dot(R_roll_x, array([[1],[0],[0]])))))
    y_axis_imu = (dot(R_yaw_z, dot(R_pitch_y, dot(R_roll_x, array([[0],[1],[0]])))))
    z_axis_imu = (dot(R_yaw_z, dot(R_pitch_y, dot(R_roll_x, array([[0],[0],[1]])))))
    
    x_test.axis = x_axis_imu
    y_test.axis = y_axis_imu
    z_test.axis = z_axis_imu

def processIMU2_message(rawMsg):

    roll2 = rawMsg.data[0]
    pitch2 = rawMsg.data[1]
    yaw2 = rawMsg.data[2] * 0

    R_yaw_z2 = array([[cos(yaw2), sin(yaw2), 0], [-sin(yaw2), cos(yaw2), 0], [0, 0, 1]])
    R_pitch_y2 = array([[cos(pitch2), 0, sin(pitch2)],[0, 1, 0],[-sin(pitch2), 0, cos(pitch2)]])
    R_roll_x2 = array([[1, 0, 0],[0, cos(roll2), -sin(roll2)],[0, sin(roll2), cos(roll2)]])

    x_axis_imu2 = dot(R_yaw_z2, dot(R_pitch_y2, dot(R_roll_x2, array([[1],[0],[0]]))))
    y_axis_imu2 = dot(R_yaw_z2, dot(R_pitch_y2, dot(R_roll_x2, array([[0],[1],[0]]))))
    z_axis_imu2 = dot(R_yaw_z2, dot(R_pitch_y2, dot(R_roll_x2, array([[0],[0],[1]]))))

    x_test_2.axis = x_axis_imu2
    y_test_2.axis = y_axis_imu2
    z_test_2.axis = z_axis_imu2

sub1 = rospy.Subscriber('imu_plate_3', float32_3, processIMU1_message)
sub2 = rospy.Subscriber('imu_pendu_3', float32_3, processIMU2_message)