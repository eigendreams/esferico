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
from kfilter import *
from limit   import *
from pid     import *
from encoder import *
################################################################################
from numpy import *
from numpy.linalg import inv 
################################################################################
#
class Motor:
    #
    def __init__(self, node_name_default = 'motor_node'):
        #
        rospy.init_node(node_name_default)
        self.nodename = rospy.get_name()
        rospy.loginfo("motor_node starting with name %s", self.nodename)
        self.rate = float(rospy.get_param("param_global_rate", '10'))
        #
        # pid_out da el valor de salida al motor de -100 a 100
        # pid_ang da el valor del angulo del encoder en su marco de referencia (lafrom std_msgs.msg import Bool IMU da el angulo de la esfera, en general, dado
        # que cada motor puede dar una cantidad potencialmente infinita de vueltas, no es posible hacer una correspondencia uno a uno
        # con el angulo real del motor. Los encoders se consideran digitales de 0 a 1023 en una vuelta completa
        # pid_vel da el valor de la velocidad del angulo del encoder en su marco de referencia, se calcula en base al valor anterior
        #
        self.pos_settings = {'kp' : 0, 'ki' : 0, 'kd' : 0, 'km' : 0, 'umbral' : 0, 'ki_dec' : 0, 'range' : 0}
        self.vel_settings = {'kp' : 0, 'ki' : 0, 'kd' : 0, 'km' : 0, 'umbral' : 0, 'ki_dec' : 0, 'range' : 0}
        # parametros de posicion, hacer parametros posteriormente, usar dynamic
        self.pos_settings['kp'] = float(rospy.get_param('~kp_pos', '70'))           
        self.pos_settings['ki'] = float(rospy.get_param('~ki_pos',  '0'))           
        self.pos_settings['kd'] = float(rospy.get_param('~kd_pos',  '0'))           
        self.pos_settings['km'] = float(rospy.get_param('~km_pos',  '0'))           
        # variables de limitacion adicionales
        self.pos_settings['umbral'] = float(rospy.get_param('~umbral_pos', '0.5'))  
        self.pos_settings['ki_dec'] = float(rospy.get_param('~ki_dec',     '0'))  
        self.pos_settings['range']  = float(rospy.get_param('~range_pos', '70'))
        # parametros de posicion, hacer parametros posteriormente, usar dynamic
        self.vel_settings['kp'] = float(rospy.get_param('~kp_vel', '70'))           
        self.vel_settings['ki'] = float(rospy.get_param('~ki_vel',  '0'))           
        self.vel_settings['kd'] = float(rospy.get_param('~kd_vel',  '0'))           
        self.vel_settings['km'] = float(rospy.get_param('~km_vel',  '0'))           
        # variables de limitacion adicionales
        self.vel_settings['umbral'] = float(rospy.get_param('~umbral_vel', '0.5'))  
        self.vel_settings['ki_dec'] = float(rospy.get_param('~ki_vel',     '0'))  
        self.vel_settings['range']  = float(rospy.get_param('~range_vel', '70'))
        # creando los objetos PID
        self.pid_pos = PID(self.pos_settings)
        self.pid_vel = PID(self.vel_settings)
        #
        # filtro de entradas del encoder
        self.kf_settings = {'Q' : 10, 'R' : 10, 'P0' : 10}
        self.filter = Kfilter(self.kf_settings)
        #
        # objeto de lctura del encoder
        self.enc_settings = {'offset' : -1}
        self.encoder = Encoder(self.enc_settings)
        #
        # Objeto de limitacion de la salida a los motores
        self.lim_settings = {'output_limit_max' : 100, 'rate' : 10, 'a' : 10, 'b' : 2}
        self.limit = Limit(self.lim_settings)
        #
        self.des = 0
        self.angle = 0
        self.speed = 0
        self.accel = 0
        #
        # Asociaciones con publicadores y suscriptores
        self.motor_out = rospy.Publisher("out", Int16)  # salida al motor
        self.encodr_in = rospy.Subscriber("in", Int16, self.ENCcallback)  # entrada del encoder
        self.desir_val = rospy.Subscriber("des", Float32, self.DEScallback)   # valor deseado de velocidad
        #
        self.srv = Server(PIDConfig, self.SRVcallback)
        #
        #
    def SRVcallback(self, config, level):
        #
        rospy.loginfo("""Reconfiugre Request: {kp_pos}, {ki_pos}, {kd_pos}, {km_pos}, {umbral_pos}, {ki_dec_pos}, {range_pos}""".format(**config))
        rospy.loginfo("""Reconfiugre Request: {kp_vel}, {ki_vel}, {kd_vel}, {km_vel}, {umbral_vel}, {ki_dec_vel}, {range_vel}""".format(**config))
        #
        self.pos_settings['kp'] = float(config['kp_pos'])           
        self.pos_settings['ki'] = float(config['ki_pos'])           
        self.pos_settings['kd'] = float(config['kd_pos'])           
        self.pos_settings['km'] = float(config['km_pos'])        
        # variables de limitacion adicionales
        self.pos_settings['umbral'] = float(config['umbral_pos'])
        self.pos_settings['ki_dec'] = float(config['ki_dec_pos'])
        self.pos_settings['range']  = float(config['range_pos'])
        # parametros de posicion, hacer parametros posteriormente, usar dynamic
        self.pos_settings['kp'] = float(config['kp_vel'])          
        self.pos_settings['ki'] = float(config['ki_vel'])          
        self.pos_settings['kd'] = float(config['kd_vel'])          
        self.pos_settings['km'] = float(config['km_vel'])         
        # variables de limitacion adicionales
        self.pos_settings['umbral'] = float(config['umbral_vel'])
        self.pos_settings['ki_dec'] = float(config['ki_dec_vel'])
        self.pos_settings['range']  = float(config['range_vel'])
        #
        self.pid_pos.resetting(self.pos_settings)
        self.pid_vel.resetting(self.vel_settings)
        #
        return config
        #
    def ENCcallback(self, data):
        #
        self.value = data.data
        self.proc  = self.encoder.compute(self.value)
        self.X = self.filter.compute(self.proc)
        #
        self.angle = self.X[0, 0]
        self.speed = self.X[1, 0]
        self.accel = self.X[2, 0]
        #
    def DEScallback(self, data):
        #
        self.des = data.data
        #
        #
    def update(self):
        #
        self.out_pos = self.pid_pos.compute(0, self.angle, self.speed)
        self.out_vel = self.pid_vel.compute(self.des, self.speed, self.accel)
        #
        if (self.pos_settings['umbral'] > 0.01):
            self.final_out = self.out_pos * (1 - min(abs(self.des) / self.pos_settings['umbral'], 1)) + self.out_vel * (min(abs(self.des) / self.pos_settings['umbral'], 1))
        else:
            self.final_out = self.out_vel
        #
        self.limited_out = self.limit.compute(self.final_out)
        #
        self.motor_out.publish(self.limited_out)
        #
        rospy.loginfo("des: " + str(self.des))
        rospy.loginfo("ang: " + str(self.angle))
        rospy.loginfo("pos: " + str(self.out_pos))
        rospy.loginfo("vel: " + str(self.out_vel))
        rospy.loginfo("sum: " + str(self.final_out))
        rospy.loginfo("lim: " + str(self.limited_out))
        #
    def spin(self):
        #
        r = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            self.update()
            r.sleep()
            #
            #
if __name__ == '__main__':

    """ main """
    motor = Motor()
    motor.spin() 