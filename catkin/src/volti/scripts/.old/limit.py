#!/usr/bin/env python
# -*- coding: utf8 -*-
################################################################################
from math import *
################################################################################
from ino_mod import *
################################################################################
#
class Limit:
    #
    def __init__(self, settings = {'output_limit_max' : 100, 'rate' : 10, 'a' : 10, 'b' : 2}):
        #
        self.output_limit_max   = float(settings['output_limit_max'])
        self.rate               = float(settings['rate'])
        self.a                  = float(settings['a'])
        self.b                  = float(settings['b'])
        #
        self.output_limit       = 100
        self.output_actual      = 0
        self.output_command     = 0
        #
        self.outputs            = [0, 0, 0]
        #
        #
    def gainFcn(self, data):
        #
        return (self.a - constrain(abs(data), 0, 100) / self.b) * (1. / self.rate)
        #
        #
    def compute(self, data):
        #
        self.output_command = data
        self.gain_val = self.gainFcn(self.output_actual)
        self.output_limit = constrain(self.output_limit + self.gain_val, 0, self.output_limit_max)
        #
        if abs(self.output_command) > abs(self.output_limit):
            self.output_actual = self.output_limit * sign(self.output_command)
        else:
            self.output_actual = self.output_command
            #
        self.outputs = self.outputs[1:] + self.outputs[:1]
        self.outputs[2] = self.output_actual
        self.output_smooth = (self.outputs[0] * 0.25 + self.outputs[1] * 0.25 + self.outputs[2] * 0.5) / 3        
        #
        return self.output_smooth
        #
        #
    def resetting(self, settings):
        #
        self.output_limit_max   = settings['output_limit_max']
        self.rate               = settings['rate']
        self.a                  = settings['a']
        self.b                  = settings['b']