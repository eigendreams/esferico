#!/usr/bin/env python
# -*- coding: utf8 -*-
################################################################################
from math import *
################################################################################
from ino_mod import *
################################################################################
from numpy import *
from numpy.linalg import inv 
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
        self.A = array([[1., self.dt, 0.5 * self.dt * self.dt], [0., 1., self.dt], [0., 0., 1.]], dtype=float)
        self.H = array([[1, 0, 0] , [0, 1, 0], [0, 0, 1]], dtype=float)
        self.Q = array([[settings['Q'], 0, 0] , [0, settings['Q'], 0], [0, 0, settings['Q']]], dtype=float)
        self.R = array([[settings['R'], 0, 0] , [0, settings['R'], 0], [0, 0, settings['R']]], dtype=float)
        self.Pkm1 = array([[settings['P0'], 0, 0] , [0, settings['P0'], 0], [0, 0, settings['P0']]], dtype=float)
        self.Xk = array([[0] , [0], [0]], dtype=float)
        self.Xkm1 = array([[0] , [0], [0]], dtype=float)
        self.Xkm1m1 = array([[0] , [0], [0]], dtype=float)
        self.Y = array([[0], [0], [0]], dtype=float) # measure
        self.Ym1 = array([[0.], [0.], [0.]], dtype=float)
        self.Ym1m1 = array([[0.], [0.], [0.]], dtype=float)
        self.times = 0
        #
    def compute(self, measure):
        #
        self.times = self.times + 1
        #
        # INPUT
        #
        if (self.times < 4):
            self.Y[0, 0] = measure
            self.Y[1, 0] = 0
            self.Y[2, 0] = 0
            #
            self.Ym1[0, 0] = measure
            self.Ym1[1, 0] = 0
            self.Ym1[2, 0] = 0
            #
            self.Ym1m1[0, 0] = measure
            self.Ym1m1[1, 0] = 0
            self.Ym1m1[2, 0] = 0
            #
            self.Xk[0, 0] = measure
            self.Xk[1, 0] = 0
            self.Xk[2, 0] = 0
            #
            self.Xkm1[0, 0] = measure
            self.Xkm1[1, 0] = 0
            self.Xkm1[2, 0] = 0
            #
            self.Xkm1m1[0, 0] = measure
            self.Xkm1m1[1, 0] = 0
            self.Xkm1m1[2, 0] = 0
        #
        self.Ym1m1[0, 0] = self.Ym1[0, 0]
        self.Ym1m1[1, 0] = self.Ym1[1, 0]
        self.Ym1m1[2, 0] = self.Ym1[2, 0]
        #
        self.Ym1[0, 0] = self.Y[0, 0]
        self.Ym1[1, 0] = self.Y[1, 0]
        self.Ym1[2, 0] = self.Y[2, 0]
        #
        self.Xkm1m1[0, 0] = self.Xkm1[0, 0]
        self.Xkm1m1[1, 0] = self.Xkm1[1, 0]
        self.Xkm1m1[2, 0] = self.Xkm1[2, 0]
        #
        self.Xkm1[0, 0] = self.Xk[0, 0]
        self.Xkm1[1, 0] = self.Xk[1, 0]
        self.Xkm1[2, 0] = self.Xk[2, 0]
        #
        # TIME UPDATE
        #
        self.Xkp  = dot(self.A , self.Xkm1)
        self.Pkp  = dot(self.A , dot(self.Pkm1, self.A.T)) + self.Q
        #
        # UPDATE
        #
        self.Y[0, 0] = measure
        self.Y[1, 0] = (measure - self.Ym1[0, 0]) / self.dt
        self.Y[2, 0] = (self.Ym1[1, 0] - self.Ym1m1[1, 0]) / self.dt
        #
        # MEASURMENT UPDATE
        #
        self.Vk    = self.Y - dot(self.H, self.Xkp)
        #
        self.Sk    = dot(self.H, dot(self.Pkp, self.H.T)) + self.R
        #
        self.Kk    = dot(self.Pkp, dot(self.H.T, inv(self.Sk)))
        #
        self.Xk    = self.Xkp + dot(self.Kk, self.Vk)
        self.Pk    = self.Pkp - dot(self.Kk, dot(self.Sk, self.Kk.T))
        #
        self.Pkm1[0, 0] = self.Pk[0, 0]
        self.Pkm1[1, 0] = self.Pk[1, 0]
        self.Pkm1[2, 0] = self.Pk[2, 0]
        self.Pkm1[0, 1] = self.Pk[0, 1]
        self.Pkm1[1, 1] = self.Pk[1, 1]
        self.Pkm1[2, 1] = self.Pk[2, 1]
        self.Pkm1[0, 2] = self.Pk[0, 2]
        self.Pkm1[1, 2] = self.Pk[1, 2]
        self.Pkm1[2, 2] = self.Pk[2, 2]
        #
        return self.Xk
        #
    def resetting(self, settings):
        #
        self.Q = array([[settings['Q'], 0, 0] , [0, settings['Q'], 0], [0, 0, settings['Q']]], dtype=float)
        self.R = array([[settings['R'], 0, 0] , [0, settings['R'], 0], [0, 0, settings['R']]], dtype=float)
        self.P0 = array([[settings['P0'], 0, 0] , [0, settings['P0'], 0], [0, 0, settings['P0']]], dtype=float)