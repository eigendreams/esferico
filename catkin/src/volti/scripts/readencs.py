#!/usr/bin/env python
# -*- coding: utf8 -*-

import rospy
import time
usleep = lambda x: time.sleep(x/1000000.0)
from std_msgs.msg import Int16
import Adafruit_BBIO.GPIO as GPIO

class Read_encoders:
    #
    def __init__(self, node_name_override = 'read_encoders'):
		#
        rospy.init_node(node_name_override)
        self.nodename = rospy.get_name()
        rospy.loginfo("Node starting with name %s", self.nodename) 
        self.rate = float(rospy.get_param("rate", 10))
        # Pines de los encoders
        self.cs1 = "P9_31"
        self.cs2 = "P9_29"
        self.do  = "P9_25"
        self.clk = "P9_23"
        #
        GPIO.cleanup()
        GPIO.setup(self.cs1, GPIO.OUT)
        GPIO.setup(self.cs2, GPIO.OUT)
        GPIO.setup(self.do,  GPIO.IN)
        GPIO.setup(self.clk, GPIO.OUT)
        #
        self.closeComm(self.cs1)
        self.closeComm(self.cs2)
        #
        self.e1val = 0
        self.e2val = 0
        #
        self.e1Pub = rospy.Publisher("e1", Int16)
        self.e2Pub = rospy.Publisher("e2", Int16)
        #
    def readSingle(self, pincsn):
        # Lectura de la trama de datos de los encoders, se toman los primeros 10 bits (posicion absoluta)
        self.chainData = 0
        GPIO.output(pincsn,   GPIO.LOW)
        GPIO.output(self.clk, GPIO.LOW)
        for k in range(16):
            GPIO.output(self.clk, GPIO.HIGH)
            self.chainData = (self.chainData << 1) | GPIO.input(self.do)
            GPIO.output(self.clk, GPIO.LOW)
        self.closeComm(pincsn)
        return (self.chainData >> 6)
        #
    def closeComm(self, pincsn):
        #
        GPIO.output(pincsn,   GPIO.HIGH)
        GPIO.output(self.clk, GPIO.HIGH)
        #
    def update(self):
		#
        self.e1val = self.readSingle(self.cs1)
        self.e2val = self.readSingle(self.cs2)
        self.e1Pub.publish(self.e1val)
        self.e2Pub.publish(self.e2val)
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
    readencs = Read_encoders()
    readencs.spin()
    