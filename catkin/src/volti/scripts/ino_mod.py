#!/usr/bin/env python
# -*- coding: utf8 -*-
################################################################################
from math import *
################################################################################
import rospy
################################################################################
#
def map(x, in_min, in_max, out_min, out_max):
    return (x - in_min) * (out_max - out_min + 0.) / (in_max - in_min + 0.) + out_min
    #
def millis(init_time):
    return 1000. * (rospy.get_time() - init_time)
    #
def constrain(x, min, max):
    if (x > max):
        return max
    if (x < min):
        return min
    return x
    #
def sign(a):
    return int((a > 0) - (a < 0))