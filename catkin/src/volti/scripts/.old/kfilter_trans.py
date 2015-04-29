#!/usr/bin/env python
# -*- coding: utf8 -*-
################################################################################
from math import *
################################################################################
from ino_mod import *
################################################################################
from numpy import *
from numpy.linalg import inv, det
################################################################################
import rospy
################################################################################
#
"""
Implementacion burde de un sistema de filtrado kalman de una sola dimension
para reducir el ruido al estimar derivadas de cosas, en realidad, podriamos hace dos
dimensiones y estimar ademas la propia posicion
"""
#
class Kfilter:
    #
    def __init__(self, settings = {'Q':10 , 'R':10 , 'P0':10, 'rate':10}):
        #
        self.dt = 1. / float(settings['rate'])
        self.X = array([[0.], [0.], [0.]])
        self.Ym1 = array([[0.], [0.], [0.]])
        self.Ym1m1 = array([[0.], [0.], [0.]])
        self.P = diag((float(settings['P0']), float(settings['P0']), float(settings['P0'])))
        self.A = array([[1., self.dt, 0.5 * self.dt * self.dt], [0., 1., self.dt], [0., 0., 1.]], dtype=float)
        self.Q = array([[settings['Q'], 0, 0] , [0, settings['Q'], 0], [0, 0, settings['Q']]], dtype=float)
        self.Y = array([[0.], [0.], [0.]], dtype=float)
        self.H = array([[1., 0., 0.], [0., 1., 0.], [0., 0., 1.]], dtype=float)
        self.R = array([[settings['R'], 0, 0] , [0, settings['R'], 0], [0, 0, settings['R']]], dtype=float)
        #
        self.times = 0
        #
    def compute(self, measure):
        #
        self.times = self.times + 1
        #
        if (self.times < 2):
            self.Y = array([[measure], [0], [0]])
            return self.Y
            #
        # INPUT
        self.Ym1m1[0, 0] = self.Ym1[0, 0]
        self.Ym1m1[1, 0] = self.Ym1[1, 0]
        self.Ym1m1[2, 0] = self.Ym1[2, 0]
        #
        self.Ym1[0, 0] = self.Y[0, 0]
        self.Ym1[1, 0] = self.Y[1, 0]
        self.Ym1[2, 0] = self.Y[2, 0]
        # TIME UPDATE
        self.X  = dot(self.A , self.X)
        self.P  = dot(self.A , dot(self.P, self.A.T)) + self.Q
        # MEASURE
        self.Y[0, 0] = measure
        self.Y[1, 0] = (measure - self.Ym1[0, 0]) / self.dt
        self.Y[2, 0] = (self.Ym1[1, 0] - self.Ym1m1[1, 0]) / self.dt
        #
        #MEASUMENT UPDATE
        self.V    = self.Y - dot(self.H, self.X)
        self.S    = dot(self.H, dot(self.P, self.H.T)) + self.R
        self.K    = dot(self.P, dot(self.H.T, inv(self.S)))
        self.X    = self.X + dot(self.K, self.V)
        self.P    = self.P - dot(self.K, dot(self.S, self.K.T))
        #
        return self.X
        #
    def resetting(self, settings):
        #
        self.P = diag((float(settings['P0']), float(settings['P0']), float(settings['P0'])))
        self.Q = array([[settings['Q'], 0, 0] , [0, settings['Q'], 0], [0, 0, settings['Q']]], dtype=float)
        self.R = array([[settings['R'], 0, 0] , [0, settings['R'], 0], [0, 0, settings['R']]], dtype=float)
        self.dt = float(settings['rate'])