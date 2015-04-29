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
class Kfilter:
    #
    def __init__(self, settings = {'Q':10. , 'R':10. , 'P0':10. , 'rate':10.}):
        #
        self.dt = 1. / float(settings['rate'])
        self.X = array([[0.], [0.], [0.]])
        self.Ym1 = array([[0.], [0.], [0.]])
        self.Ym1m1 = array([[0.], [0.], [0.]])
        self.P = diag((float(settings['P0']), float(settings['P0']), float(settings['P0'])))
        self.A = array([[1., self.dt, 0.5 * self.dt * self.dt], [0., 1., self.dt], [0., 0., 1.]])
        self.Q = array([[settings['Q'], 0., 0.] , [0., settings['Q'], 0.], [0., 0., settings['Q']]])
        self.Y = array([[0.], [0.], [0.]])
        self.H = array([[1., 0., 0.], [0., 1., 0.], [0., 0., 1.]])
        self.R = array([[settings['R'], 0., 0.] , [0., settings['R'], 0.], [0., 0., settings['R']]])
        #
        self.times = 0
        #
    def kf_predict(self, _X, _P, _A, _Q):
        _X = dot(_A, _X)
        _P = dot(_A, dot(_P, _A.T)) + _Q
        return (_X, _P)
        #
    def kf_update(self, _X, _P, _Y, _H, _R):
        _IM = dot(_H, _X)
        _IS = _R + dot(_H, dot(_P, _H.T))
        _K  = dot(_P, dot(_H.T, inv(_IS)))
        _X  = _X + dot(_K, (_Y - _IM))
        _P  = _P - dot(_K, dot(_IS, _K.T))
        #_LH = self.gauss_pdf(_Y, _IM, _IS)
        #return (_X, _P, _K, _IM, _IS, _LH)
        return (_X, _P, _K, _IM, _IS)
        #
    def gauss_pdf(self, _X, _M, _S):
        if _M.shape[1] == 1:
            _DX = _X - tile(_M, _X.shape[1])
            _E = 0.5 * sum(_DX * (dot(inv(_S), _DX)), axis=0)
            _E = _E + 0.5 * _M.shape[0] * log(2 * pi) + 0.5 * log(det(_S))
            _P = exp(-_E)
        elif _X.shape[1] == 1:
            _DX = tile(_X, _M.shape[1])- _M
            _E = 0.5 * sum(_DX * (dot(inv(_S), _DX)), axis=0)
            _E = _E + 0.5 * _M.shape[0] * log(2 * pi) + 0.5 * log(det(_S))
            _P = exp(-_E)
        else:
            _DX = _X-_M
            _E = 0.5 * dot(_DX.T, dot(inv(_S), _DX))
            _E = _E + 0.5 * _M.shape[0] * log(2 * pi) + 0.5 * log(det(_S))
            _P = exp(-_E)
        return (_P[0], _E[0]) 
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
        (self.X, self.P) = self.kf_predict(self.X, self.P, self.A, self.Q)
        #
        self.Y[0, 0] = measure
        self.Y[1, 0] = (measure - self.Ym1[0, 0]) / self.dt
        self.Y[2, 0] = (self.Ym1[1, 0] - self.Ym1m1[1, 0]) / self.dt
        #
        #(self.X, self.P, self.K, self.IM, self.IS, self.LH) = self.kf_update(self.X, self.P, self.Y, self.H, self.R)
        (self.X, self.P, self.K, self.IM, self.IS) = self.kf_update(self.X, self.P, self.Y, self.H, self.R)
        #
        return self.X
        #
    def resetting(self, settings):
        #
        self.P = diag((float(settings['P0']), float(settings['P0']), float(settings['P0'])))
        self.Q = array([[settings['Q'], 0., 0.] , [0., settings['Q'], 0.], [0., 0., settings['Q']]])
        self.R = array([[settings['R'], 0., 0.] , [0., settings['R'], 0.], [0., 0., settings['R']]])
        self.dt = 1 / float(settings['rate'])
