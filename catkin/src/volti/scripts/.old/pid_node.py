#!/usr/bin/env python
# -*- coding: utf8 -*-
################################################################################
from math import *
################################################################################
import rospy
from std_msgs.msg import Bool
from std_msgs.msg import Int16
from std_msgs.msg import Float32
from dynamic_reconfigure.server import Server
################################################################################
from volti.cfg import PIDConfig
################################################################################
from ino_mod import *
################################################################################

class PID_node:

    def __init__(self, node_name_default = 'pid_node'):

        rospy.init_node(node_name_default)
        self.nodename = rospy.get_name()
        rospy.loginfo("pid_node starting with name %s", self.nodename)
        self.rate = float(rospy.get_param("param_global_rate", '10'))

        # pid_out da el valor de salida al motor de -100 a 100
        # pid_ang da el valor del angulo del encoder en su marco de referencia (lafrom std_msgs.msg import Bool IMU da el angulo de la esfera, en general, dado
        # que cada motor puede dar una cantidad potencialmente infinita de vueltas, no es posible hacer una correspondencia uno a uno
        # con el angulo real del motor. Los encoders se consideran digitales de 0 a 1023 en una vuelta completa
        # pid_vel da el valor de la velocidad del angulo del encoder en su marco de referencia, se calcula en base al valor anterior
        self.pidOutPub = rospy.Publisher("~pid_out", Int16)
        self.pidAngPub = rospy.Publisher("~pid_ang", Float32)
        self.pidVelPub = rospy.Publisher("~pid_vel", Float32)

        # parametros de posicion, hacer parametros posteriormente, usar dynamic
        self.kp_pos = float(rospy.get_param('~kp_pos', '70'))           # kp
        self.ki_pos = float(rospy.get_param('~ki_pos',  '0'))           # ki
        self.kd_pos = float(rospy.get_param('~kd_pos',  '0'))           # kd
        self.km_pos = float(rospy.get_param('~km_pos',  '0'))           # km
        # variables de limitacion adicionales
        self.umbral_pos = float(rospy.get_param('~umbral_pos', '0.5'))  # valor en radianes dentro del cual el error de posicion se toma nulo
        self.range_pos  = float(rospy.get_param('~range_pos', '70'))    # maximo pwm permitido en este modo de control
        self.kierr_pos  = float(rospy.get_param('~kierr_pos',  '1.2'))  # rango en radianes DENTRO del cual acumular error integral
        self.kimax_pos  = float(rospy.get_param('~kimax_pos', '10'))    # macima pwm contribuido por el error integral
        self.kisum_pos  = 0                                             # suma del error integral
        self.error_pos  = 0                                             # valor del error de posicion

        # variables de velocidad, hacer parametros posteriormente, usar dynamic
        self.kp_vel = float(rospy.get_param('kp_vel', '70'))
        self.ki_vel = float(rospy.get_param('~ki_vel',  '0'))
        self.kd_vel = float(rospy.get_param('~kd_vel',  '0'))
        self.km_vel = float(rospy.get_param('~km_vel',  '0'))
        # variables de limitacion adicionales
        self.umbral_vel = float(rospy.get_param('~umbral_vel', '0.5'))
        self.range_vel  = float(rospy.get_param('~range_vel', '70'))    
        self.kierr_vel  = float(rospy.get_param('~kierr_vel',  '1.2'))
        self.kimax_vel  = float(rospy.get_param('~kimax_vel', '10'))    
        self.kisum_vel  = 0                                             
        self.error_vel  = 0                 

        # guardar el valor inicial a usar como posible referencia para determinar cuanto hay que avanzar el pendulo.
        # como en el finder, como se hace control de velocidad, al final solo nos importa mantenernos en una posicion establecida
        # pero controlando como llegamos ahi, probar...
        self.pid_offset = 0                         

        # variables auxilieres
        self.pid_out = 0
        self.pid_ang = 0
        self.pid_vel = 0

        # variables auxiliares
        self.pid_lec = 0                # lectura del encoder, de 0 a 1023 (SIEMPRE DIGITAL)
        self.pid_ang_rng = 0                    
        self.pid_lec_dst = 0
        self.pid_ang_lst = 0
        self.pid_ang_chg = 0
        self.pid_ang_abs = 0
        self.pid_ang_lap = 0
        self.pid_offset_internal = 0.
        self.pid_ang_tmp_internal = 0      
        self.pid_ang_lst_internal = 0
        self.pid_ang_abs_internal = 0

        self.times = 0                      
        self.times_lec = 0

        self.init_time = rospy.get_time()

        self.pidResetSub = rospy.Subscriber("~pid_reset", Int16, self.pidResetCb)
        self.pidLecSub = rospy.Subscriber("~pid_lec", Int16, self.pidLecCb)
        self.pidDesSub = rospy.Subscriber("~pid_des", Float32, self.pidDesCb)
        self.offsetSub = rospy.Subscriber("~offset", Int16, self.offsetCb)

        self.srv = Server(PIDConfig, self.callback)


    def callback(self, config, level):

        rospy.loginfo("""Reconfiugre Request: {kp_pos}, {ki_pos}, {kd_pos}, {km_pos}, {umbral_pos}, {range_pos}, {kierr_pos}, {kimax_pos}""".format(**config))
        rospy.loginfo("""Reconfiugre Request: {kp_vel}, {ki_vel}, {kd_vel}, {km_vel}, {umbral_vel}, {range_vel}, {kierr_vel}, {kimax_vel}""".format(**config))

        self.kp_pos     = config['kp_pos']
        self.ki_pos     = config['ki_pos']
        self.kd_pos     = config['kd_pos']
        self.km_pos     = config['km_pos']
        self.umbral_pos = config['umbral_pos']
        self.range_pos  = config['range_pos']    
        self.kierr_pos  = config['kierr_pos']
        self.kimax_pos  = config['kimax_pos'] 

        self.kp_vel     = config['kp_vel']
        self.ki_vel     = config['ki_vel']
        self.kd_vel     = config['kd_vel']
        self.km_vel     = config['km_vel']
        self.umbral_vel = config['umbral_vel']
        self.range_vel  = config['range_vel']    
        self.kierr_vel  = config['kierr_vel']
        self.kimax_vel  = config['kimax_vel']   

        return config


    # regresa al angulo inicial haciendo que ang_des sea igual a cero
    def pidResetCb(self, data):

        if (data.data == 1):
            self.pid_ang_des = 0


    # reset de todas las variables temporales, reinicio suave del nodo
    def offsetCb(self, data):

        if (data.data == 1):
            self.pid_offset  = self.pid_lec
            self.pid_ang_tmp = 0
            self.pid_ang_lst = 0
            self.pid_ang_abs = 0
            self.pid_ang = 0
            self.pid_ang_lap =  0
            self.pid_ang_lap_lst = 0


    def pid_pos(self):

        if (abs(self.pid_ang_des - self.pid_ang) < self.umbral_pos):
           self.error_pos = 0.
        else:
            self.error_pos = self.pid_ang_des - self.pid_ang

        if (abs(self.pid_ang_des - self.pid_ang) < self.kierr_pos):
            if (abs(self.pid_ang_des - self.pid_ang) < self.umbral_pos):
                self.kisum_pos = 0.;
            else:
                self.kisum_pos += self.ki_pos * self.error_pos
                self.kisum_pos = self.constrain(self.kisum_pos, -self.kimax_pos, self.kimax_pos)
        else:
            self.kisum_pos = 0.

        self.pid_out = self.constrain(self.kp_pos * self.error_pos + self.kisum_pos - self.kd_pos * (self.pid_ang - self.pid_ang_lst) + self.km_pos * self.pid_ang_des, -self.range_pos, self.range_pos)


    def pid_vel(self):

        if (abs(self.pid_vel_des - self.pid_vel) < self.umbral_vel):
           self.error_vel = 0.
        else:
            self.error_vel = self.pid_vel_des - self.pid_vel + 0.

        if (abs(self.pid_vel_des - self.pid_vel) < self.kierr_vel):
            if (abs(self.pid_vel_des - self.pid_vel) < self.umbral_vel):
                self.kisum_vel = 0.;
            else:
                self.kisum_vel += self.ki_vel * self.error_vel
                self.kisum_vel = self.constrain(self.kisum_vel, -self.kimax_vel, self.kimax_vel)
        else:
            self.kisum_vel = 0.

        self.pid_out = self.constrain(self.kp_vel * self.error_vel + self.kisum_vel - self.kd_pos * (self.pid_ang - self.pid_ang_lst) + self.km_vel * self.pid_vel_des, -self.range_vel, self.range_vel)


    def angCalc(self):

        self.times += 1

        self.pid_offset_internal  = self.map(self.pid_offset, 0., 1023., 0, 2 * pi)
        self.pid_ang_tmp_internal = self.map(self.pid_lec, 0., 1023., 0, 2 * pi)

        self.pid_ang_lst_internal = self.pid_ang_abs_internal
        self.pid_ang_abs_internal = self.pid_ang_tmp_internal

        if (self.times > 4):
            # encuentra si el cambio fue de 0 a 2pi
            if (self.pid_ang_abs_internal > 1.7 * pi and self.pid_ang_lst_internal < 0.3 * pi):
                self.pid_ang_lap -= 1
            # encuentra si el cambio fue de 2pi a 0
            if (self.pid_ang_abs_internal < 0.3 * pi and self.pid_ang_lst_internal > 1.7 * pi):
                self.pid_ang_lap += 1

        self.pid_ang_lap_lst = self.pid_ang
        self.pid_ang = 2 * pi * self.pid_ang_lap + self.pid_ang_abs_internal - self.pid_offset_internal 
        self.pid_vel = 10 * (self.pid_ang - self.pid_ang_lap_lst)


    def pidLecCb(self, data):

        self.times_lec += 1
        if (self.times_lec < 4):
            self.offset = data.data
        self.pid_lec = data.data

        self.angCalc()

        if (abs(self.pid_vel_des) < 0.05):
            # self.pid_ang_des = self.constrain(self.pid_ang_des, 0, 1000)
            self.pid_pos()
            # print "angdes " + str(self.pid_ang_des)
        else:
            self.pid_ang_des = self.pid_ang
            self.pid_vel()


    def update(self):

        self.pidOutPub.publish(self.pid_out)
        self.pidAngPub.publish(self.pid_ang)
        self.pidVelPub.publish(self.pid_vel)

        print("kp_vel: " + str(self.kp_vel) + " ki_vel: " + str(self.ki_vel) + " kd_vel: " + str(self.kd_vel) + " km_vel: " + str(self.km_vel))


    def spin(self):

        r = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            self.update()
            r.sleep()


if __name__ == '__main__':

    """ main """
    pid_node = PID_node()
    pid_node.spin()
