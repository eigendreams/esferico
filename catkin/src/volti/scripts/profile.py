#!/usr/bin/env python
# -*- coding: utf8 -*-

################################################################################
from math import *
################################################################################
from ino_mod import *
################################################################################
import rospy
from std_msgs.msg import Int16
from std_msgs.msg import Float32
################################################################################

class Profile:
    #
    def __init__(self, settings = {'max_output' : 30, 'max_speed' : 100, 'rate' : 10, 'heal_time_at_0pc' : 20, 'stable_point_pc' : 20}):
        #
        self.max_output         = float(settings['max_output'])
        self.max_speed          = float(settings['max_speed'])
        self.rate               = float(settings['rate'])
        self.heal_time_at_0pc   = float(settings['heal_time_at_0pc'])
        self.stable_point_pc    = float(settings['stable_point_pc'])
        #
        self.output_actual      = 0.
        self.output_des         = 0.
        self.last_output_actual = 0.
        self.last_output_des    = 0.
        self.output_limit       = 0.
        #
        self.x_extend           = self.stable_point_pc
        self.y_extend           = -100. / self.heal_time_at_0pc
        self.m_signed           = self.y_extend / self.x_extend
        self.y0_offset          = 100. / self.heal_time_at_0pc
        #
    def gainFcn(self, data):
        #
        return (self.y0_offset + self.m_signed * abs(data))
        #
    def compute(self, data):
        #
        self.last_output_actual = self.output_actual
        self.output_des = data
        self.gain_val = self.gainFcn(self.output_actual) / self.rate
        self.output_limit = constrain(self.output_limit + self.gain_val, 0, self.max_output)
        #
        if abs(self.output_des) > abs(self.output_limit):
            self.output_actual = self.output_limit * sign(self.output_des)
        else:
            self.output_actual = self.output_des
        #
        self.output_change = self.output_actual - self.last_output_actual
        #
        if (abs(self.output_change) > self.max_speed / self.rate):
            #
            self.output_actual = constrain(self.output_actual, self.last_output_actual - self.max_speed / self.rate, self.last_output_actual + self.max_speed / self.rate)
        #
        return self.output_actual
        #
    def resetting(self, settings):
        #
        self.max_output         = float(settings['max_output'])
        self.max_speed          = float(settings['max_speed'])
        self.rate               = float(settings['rate'])
        self.heal_time_at_0pc   = float(settings['heal_time_at_0pc'])
        self.stable_point_pc    = float(settings['stable_point_pc'])
        #
        self.x_extend           = self.stable_point_pc
        self.y_extend           = -100. / self.heal_time_at_0pc
        self.m_signed           = self.y_extend / self.x_extend
        self.y0_offset          = 100. / self.heal_time_at_0pc
        #