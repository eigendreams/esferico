#!/usr/bin/env python
# -*- coding: utf8 -*-
################################################################################
from math import *
################################################################################
from ino_mod import *
################################################################################
#
"""
Tratar de pasar de un sistema de encoders digitales de 0 a 1023 a un valor continuo en radianes que
podria no tener limites, la primera lectura, por la disposicion mecanica, se toma como un cero de angulo,
pero lo que se quiera controlar del motor sera velocidad, y de esta forma no deberia ser problematico con la ayuda
de los sensores externos como la IMU.
"""
#
class Encoder:
    #
    def __init__(self, settings = {'offset' : -1}):
        #
        self.offset = settings['offset']
        self.times  = 0
        self.angle  = 0
        self.laps   = 0
        self.output_angle = 0
        self.corrected_change = 0
        #
    def compute(self, measure):
        #
        if (measure == -1):
            #
            self.output_angle = self.output_angle + self.corrected_change
            return self.output_angle
        #
        if (self.times == 0):
            self.internal_angle_last    = map(measure, 0., 1023., 0., 2. * pi)
            self.internal_angle         = map(measure, 0., 1023., 0., 2. * pi)
            if (self.offset < 0):
                self.internal_offset    = map(measure, 0., 1023., 0., 2. * pi)
            else:
                self.internal_offset    = map(self.offset, 0., 1023., 0., 2. * pi)
        #
        self.internal_angle_last        = self.internal_angle
        self.internal_angle             = map(measure, 0., 1023., 0., 2. * pi)
        #
        self.change = self.internal_angle - self.internal_angle_last
        if (abs(self.change) > pi): # es un cambio muy drastico, asumiremos que hubo un salto,  max RPM de 300 a 20 hz
            if (self.internal_angle_last > self.internal_angle): # ej saltar de 900 a 100 -> 100 -800 -> change de 800 
                self.laps = self.laps + 1
            if (self.internal_angle_last <= self.internal_angle): # ej saltar de 100 a 900 -> 900 - 100 -> change de 800
                self.laps = self.laps - 1
        #
        self.output_angle_last          = self.output_angle
        self.output_angle               = 2 * pi * self.laps + self.internal_angle - self.internal_offset
        #
        self.corrected_change           = self.output_angle - self.output_angle_last
        #
        self.times                      = self.times + 1
        #
        return self.output_angle
        #
    def resetting(self, settings):
        #
        self.offset = settings['offset']