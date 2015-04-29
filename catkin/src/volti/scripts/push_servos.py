#!/usr/bin/env python
# -*- coding: utf8 -*-

import rospy
import time
import serial
from ino_mod import *
from std_msgs.msg import Int16

class Push_servos:
    #
    def __init__(self, node_name_override = 'push_servos'):
		#
        rospy.init_node(node_name_override)
        self.nodename = rospy.get_name()
        rospy.loginfo("Node starting with name %s", self.nodename) 
        self.rate = float(rospy.get_param("rate", 10))
        #
        self.ser = serial.Serial('/dev/ttyO4', 115200)
        #
        self.m1val = 0
        self.m2val = 0
        self.alval = 0
        # 0x7530 = 30000 usado como identificador de inicio de la trama
        # id, id, al, al, m1, m1, m2, m2, sum, sum
        self.datar = [0x75,0x30,0,0,0,0,0,0,0,0]
        #
        self.inittime = rospy.get_time()
        self.timelastal = -1000
        self.timelastnal = -1000
        #
        self.m1sub = rospy.Subscriber("m1", Int16, self.m1cb)
        self.m2sub = rospy.Subscriber("m2", Int16, self.m2cb)
        self.alsub = rospy.Subscriber("al", Int16, self.alcb)
        #
    def m1cb(self,data):
        #
        self.m1val = data.data
        #
    def m2cb(self, data):
        #
        self.m2val = data.data
        #
    def alcb(self, data):
        #
        self.timelastal = millis(self.inittime)
        self.alval = data.data
        #
    def update(self):
		# if al has not been received, shtudown
        if ((millis(self.inittime) - self.timelastal) > 2000):
            self.alval = 0
            self.m1val = 0
            self.m2val = 0
            self.timelastnal = millis(self.inittime)
        # wait a little after each disconnect to avoid jitter
        if ((millis(self.inittime) - self.timelastnal) < 2000):
            self.alval = 0
            self.m1val = 0
            self.m2val = 0
        #
        self.datar[2]  = (self.alval >> 8) & 255
        self.datar[3]  = (self.alval >> 0) & 255
        self.datar[4]  = (self.m1val >> 8) & 255
        self.datar[5]  = (self.m1val >> 0) & 255
        self.datar[6]  = (self.m2val >> 8) & 255
        self.datar[7]  = (self.m2val >> 0) & 255
        self.datar[8]  = ((self.alval + self.m1val + self.m2val) >> 8) & 255
        self.datar[9]  = ((self.alval + self.m1val + self.m2val) >> 0) & 255
        #
        self.ser.write(self.datar)
        #
    def spin(self):
		#
        r = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            self.update()
            r.sleep()
        #   
if __name__ == '__main__':
    """ main """
    push_servos = Push_servos()
    push_servos.spin()
    