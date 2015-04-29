#!/usr/bin/env python
# -*- coding: utf8 -*-

"""
Esta clase trata de limitar la posibilidad de quemar los motores de DC del robot, para cualquier clase
de robot donde ello sea un posible problema.

Nodo:   motor_output_limit
    Suscriptores:   motorin     Valor de salida PWM al motor dada por el controlador, de -100 a 100
    Publicadores:   motorout    Valor de salida PWM al motor limitada, hasta -100 a 100

Una funcion de ganancia actualiza el valor de salida limite, output_limit, dependiendo del valor de PWM
actual en cada sampleo. Ej, si estamos en salida 0, cada segundo aumenta un 10 la salida maxima al motor,
si es de 100%, cada segundo disminuye un 40 la salida maxima al motor. Con limites fijos entre 0 y 100
"""

################################################################################
from math import *
################################################################################
from ino_mod import *
################################################################################
import rospy
from std_msgs.msg import Int16
from std_msgs.msg import Float32
################################################################################

class Motor_limit:

    def __init__(self, node_name_override = 'motor_output_limit'):
        rospy.init_node(node_name_override)
        self.nodename = rospy.get_name()
        rospy.loginfo("motor_output_limit starting with name %s", self.nodename) 
        self.rate = rospy.get_param("param_global_rate", 10)
        self.init_time = rospy.get_time()

        self.output_limit_max   = 100;
        self.output_limit       = 100;
        self.output_actual      = 0;
        self.output_command     = 0;

        self.pre_time           = 0;
        self.now_time           = 0;
        self.times              = 0;
        self.s_rate             = 10;
        self.c_rate             = 10;

        self.motorout  = rospy.Publisher("output", Int16)
        self.motorin   = rospy.Subscriber("input", Int16, self.motorcb)

    def gainFcn(self, data):
        return (1. - constrain(abs(data), 0, 100) / 20.) * (10. / self.s_rate)

    def motorcb(self, data):
        self.now_time = millis(self.init_time);

        # time stuff, default to 100 ms
        if (self.times == 0):
            self.pre_time = self.now_time - 100;
        self.times = self.times + 1;

        print(self.output_limit)

        # else, 50ms de offset, no entiendo bien porque... pero funciona, TO TEST
        self.c_rate = 950. / (self.now_time - self.pre_time)
        self.s_rate = self.s_rate * 0.9 + self.c_rate * 0.1
        self.pre_time = self.now_time

        self.output_command = data.data
        self.gain_val = self.gainFcn(self.output_actual)
        self.output_limit = constrain(self.output_limit + self.gain_val, 0, self.output_limit_max)
        
        if abs(self.output_command) > abs(self.output_limit):
            self.output_actual = self.output_limit * sign(self.output_actual)
        else:
            self.output_actual = self.output_command

        self.update();

    def update(self):
        self.motorout.publish(self.output_actual)

    def spin(self):
        r = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            #self.update()
            r.sleep()

if __name__ == '__main__':
    """ main """
    motor_output_limit = Motor_output_limit()
    motor_output_limit.spin()
