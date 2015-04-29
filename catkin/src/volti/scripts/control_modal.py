#!/usr/bin/env python
# -*- coding: utf8 -*-

import rospy

from std_msgs.msg import Bool
from std_msgs.msg import Int16
from std_msgs.msg import Float32
from sensor_msgs.msg import Joy

from ino_mod import *

class Control_interface:
    #
    def __init__(self, node_name_override = 'control_interface'):
		#
        rospy.init_node(node_name_override)
        self.nodename = rospy.get_name()
        rospy.loginfo("Node starting with name %s", self.nodename) 
        self.rate = float(rospy.get_param("rate", 5))
        # Nombres de los botones de Joy
        self.axes_names     = {'left_stick_hor':0, 'left_stick_ver':1, 'LT':2, 'right_stick_hor':3, 'right_stick_ver':4, 'RT':5, 'cross_hor':6, 'cross_ver':7}
        self.buttons_names  = {'A':0, 'B':1, 'X':2, 'Y':3, 'LB':4, 'RB':5, 'back':6, 'start':7, 'power':8, 'btn_stick_left':9, 'btn_stick_right':10}
        # Publicadores a los motores, SOLO para el modo manual
        self.m1    = rospy.Publisher("m1", Int16)              # salida al motor 1
        self.m2    = rospy.Publisher("m2", Int16)              # salida al motor 1
        # Publicadores para valores controlados, bandera de comunicacion, y modo de control (0 automatico, 1 manual)
        self.veldespub  = rospy.Publisher("vel_delante_des", Float32)
        self.angdespub  = rospy.Publisher("ang_lateral_des", Float32)
        self.alpub      = rospy.Publisher("al", Int16)
        self.conmode    = rospy.Publisher("con_mode", Int16)
        #
        self.btog = 0
        self.lastb = 0
        self.lastbtog = 0
        self.btogchanged = 0
        #
        self.powtog = 0
        self.lastpow = 0
        self.lastpowtog = 0
        self.powtogchanged = 0
        #
        self.lastrb = 0
        self.lastlb = 0
        self.rtval = 0
        self.ltval = 0
        #
        self.angle_des = 0
        self.vel_des    = 0
        # Proteccion por timeout de Joy
        self.inittime = rospy.get_time()
        self.timelastjoy = -1000
        self.timed_out = True
        #
        self.joySub = rospy.Subscriber("joy", Joy, self.joyCb)       
        #
    def joyCb(self, data):
        #
        self.timelastjoy = millis(self.inittime)
        self.timed_out   = False
        # Quiero detectar cambion del boton de power
        self.lastpowtog = self.powtog
        if (data.buttons[self.buttons_names['power']] is 1 and self.lastpow is 0):
            self.powtog = int(not self.powtog)
        self.powtogchanged = self.lastpowtog or not (self.powtog == self.lastpowtog)
        self.lastpow = data.buttons[self.buttons_names['power']]
        # y del boton b
        self.lastbtog = self.btog
        if (data.buttons[self.buttons_names['B']] is 1 and self.lastb is 0):
            self.btog = int(not self.btog)
        self.btogchanged = 0 or not (self.btog == self.lastbtog)
        self.lastb = data.buttons[self.buttons_names['B']]
        # Corrigiendo escala y signo de los gatillos
        self.rtval = (data.axes[self.axes_names['RT']] - 1) / -2.
        self.ltval = (data.axes[self.axes_names['LT']] - 1) / -2.
        #
        self.lastlb = data.buttons[self.buttons_names['LB']]
        self.lastrb = data.buttons[self.buttons_names['RB']]
        #
        self.angle_des   = -data.axes[self.axes_names['left_stick_hor']]
        self.vel_des     = data.axes[self.axes_names['right_stick_ver']]
        #
    def update(self):
        # Proteccion por timeout
        if ((millis(self.inittime) - self.timelastjoy) > 1000):
            if not self.timed_out :
                self.angle_des = 0
                self.vel_des = 0
                self.powtog = 0
                self.btog = 0
            self.timed_out = True
        #
        self.conmode.publish(self.btog)
        self.alpub.publish(self.powtog)
        # Obteniendo valores de escalamiento para el modo manual
        self.angmultval = constrain( (1 + self.ltval) * (1 + self.lastlb), 1, 4)
        self.velmultval = constrain( (1 + self.rtval) * (1 + self.lastrb), 1, 4)
        #
        if (self.btog is 1):
            # Modo manual
            self.m1.publish(constrain(self.vel_des * 7 * self.velmultval * 100 - self.angle_des * 7 * self.angmultval * 100,-3000, 3000))
            self.m2.publish(constrain(self.vel_des * 7 * self.velmultval * 100 + self.angle_des * 7 * self.angmultval * 100,-3000, 3000))
        else:
            # Modo controlado
            self.angdespub.publish(self.angle_des * 1.0)
            self.veldespub.publish(self.vel_des   * 1.0)
            #
    def spin(self):
		#
        r = rospy.Rate(self.rate)
        #
        while not rospy.is_shutdown():
            self.update()
            r.sleep()
        #
if __name__ == '__main__':
    """ main """
    control_interface = Control_interface()
    control_interface.spin()
