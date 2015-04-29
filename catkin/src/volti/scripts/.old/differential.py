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
class Differential:
    #
    def __init__(self, node_name_default = 'differential_node'):
        #
        rospy.init_node(node_name_default)
        self.nodename = rospy.get_name()
        rospy.loginfo("differential_node starting with name %s", self.nodename)
        self.rate = float(rospy.get_param("param_global_rate", '10'))
        #
        self.times = 0
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
        self.pos_settings['kp'] = float(rospy.get_param('~kp_pos', '40'))           
        self.pos_settings['ki'] = float(rospy.get_param('~ki_pos',  '0'))           
        self.pos_settings['kd'] = float(rospy.get_param('~kd_pos',  '0'))           
        self.pos_settings['km'] = float(rospy.get_param('~km_pos',  '0'))           
        # variables de limitacion adicionales
        self.pos_settings['umbral'] = float(rospy.get_param('~umbral_pos', '0.05'))  
        self.pos_settings['ki_dec'] = float(rospy.get_param('~ki_dec',     '0'))  
        self.pos_settings['range']  = float(rospy.get_param('~range_pos', '20'))
        # parametros de posicion, hacer parametros posteriormente, usar dynamic
        self.vel_settings['kp'] = float(rospy.get_param('~kp_vel', '40'))           
        self.vel_settings['ki'] = float(rospy.get_param('~ki_vel',  '0'))           
        self.vel_settings['kd'] = float(rospy.get_param('~kd_vel',  '0'))           
        self.vel_settings['km'] = float(rospy.get_param('~km_vel',  '0'))           
        # variables de limitacion adicionales
        self.vel_settings['umbral'] = float(rospy.get_param('~umbral_vel', '0.05'))  
        self.vel_settings['ki_dec'] = float(rospy.get_param('~ki_vel',     '0'))  
        self.vel_settings['range']  = float(rospy.get_param('~range_vel', '20'))
        #
        # creando los objetos PID
        self.pid_pos_m1 = PID(self.pos_settings)
        self.pid_vel_m1 = PID(self.vel_settings)
        self.pid_pos_m2 = PID(self.pos_settings)
        self.pid_vel_m2 = PID(self.vel_settings)
        #
        # filtro de entradas del encoder
        self.kf_settings = {'Q' : 10, 'R' : 10, 'P0' : 10, 'rate' : 10}
        #
        # filtros kalman de los encoders
        self.filter_e1  = Kfilter(self.kf_settings)
        self.filter_e2  = Kfilter(self.kf_settings)
        #
        # objeto de lectura del encoder
        self.enc_settings = {'offset' : -1}
        #
        self.enc_1 = Encoder(self.enc_settings)
        self.enc_2 = Encoder(self.enc_settings)
        #
        # Objeto de limitacion de la salida a los motores
        self.lim_settings = {'output_limit_max' : 100, 'rate' : self.rate, 'a' : 10, 'b' : 2}
        #
        self.limit_m1 = Limit(self.lim_settings)
        self.limit_m2 = Limit(self.lim_settings)
        #
        self.des_m1 = 0
        self.angle_m1 = 0
        self.speed_m1 = 0
        self.accel_m1 = 0
        #
        self.des_m2 = 0
        self.angle_m2 = 0
        self.speed_m2 = 0
        self.accel_m2 = 0
        #
        self.angle_lock_m1 = 0
        self.angle_lock_m2 = 0
        #
        self.vf_val = 0
        self.vs_val = 0
        #
        # Asociaciones con publicadores y suscriptores
        self.m1 = rospy.Publisher("m1", Int16)  # salida al motor 1
        self.m2 = rospy.Publisher("m2", Int16)  #
        self.e1 = rospy.Subscriber("e1", Int16, self.e1cb)  # entrada del encoder 1
        self.e1vel = rospy.Publisher("e1vel", Float32)  #
        self.e1accel = rospy.Publisher("e1accel", Float32)  #
        self.e2 = rospy.Subscriber("e2", Int16, self.e2cb)  #
        #
        #
        self.vf = rospy.Subscriber("vf", Float32, self.vfcb)  # velocidad deseada adelante atras
        self.vs = rospy.Subscriber("vs", Float32, self.vscb)  # velocidad deseada a los lados
        #
        #
        self.srv = Server(PIDConfig, self.SRVcallback)
        #
        #
    def SRVcallback(self, config, level):
        #
        rospy.loginfo("""POS Reconfiugre Request: {kp_pos}, {ki_pos}, {kd_pos}, {km_pos}, {umbral_pos}, {ki_dec_pos}, {range_pos}""".format(**config))
        rospy.loginfo("""VEL Reconfiugre Request: {kp_vel}, {ki_vel}, {kd_vel}, {km_vel}, {umbral_vel}, {ki_dec_vel}, {range_vel}""".format(**config))
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
        self.vel_settings['kp'] = float(config['kp_vel'])          
        self.vel_settings['ki'] = float(config['ki_vel'])          
        self.vel_settings['kd'] = float(config['kd_vel'])          
        self.vel_settings['km'] = float(config['km_vel'])         
        # variables de limitacion adicionales
        self.vel_settings['umbral'] = float(config['umbral_vel'])
        self.vel_settings['ki_dec'] = float(config['ki_dec_vel'])
        self.vel_settings['range']  = float(config['range_vel'])
        #
        self.pid_pos_m1.resetting(self.pos_settings)
        self.pid_vel_m1.resetting(self.vel_settings)
        self.pid_pos_m2.resetting(self.pos_settings)
        self.pid_vel_m2.resetting(self.vel_settings)
        #
        return config
        #
    def e1cb(self, data):
        #
        self.value_m1 = data.data
        self.proc_m1 = self.enc_1.compute(self.value_m1)
        self.X_m1 = self.filter_e1.compute(self.proc_m1)
        #
        self.angle_m1 = self.X_m1[0, 0]
        self.speed_m1 = self.X_m1[1, 0]
        self.accel_m1 = self.X_m1[2, 0]
        #
        self.e1vel.publish(self.speed_m1)
        self.e1accel.publish(self.accel_m1)
        #
    def e2cb(self, data):
        #
        self.value_m2 = data.data
        self.proc_m2 = self.enc_2.compute(self.value_m2)
        self.X_m2 = self.filter_e2.compute(self.proc_m2)
        #
        self.angle_m2 = self.X_m2[0, 0]
        self.speed_m2 = self.X_m2[1, 0]
        self.accel_m2 = self.X_m2[2, 0]
        #
    def vfcb(self, data):
        #
        self.vf_val = data.data
        #
    def vscb(self, data):
        #
        self.vs_val = data.data
        #
    def update(self):
        #
        # velocidades deseadas
        #
        self.des_m1 = (self.vf_val + self.vs_val) / 2.
        self.des_m2 = (self.vf_val - self.vs_val) / 2.
        #
        if (self.times < 10):
            # inicializacion
            self.angle_lock_m1 = self.angle_m1
        if (self.des_m1 > self.pos_settings['umbral']):
            # no estamos manteniendo ningun angulo
            self.angle_lock_m1 = self.angle_m1
        else:
            # deseamos mantener el angulo
            pass
        #
        self.out_pos_m1 = self.pid_pos_m1.compute(self.angle_lock_m1, self.angle_m1, self.speed_m1)
        self.out_vel_m1 = self.pid_vel_m1.compute(self.des_m1, self.speed_m1, self.accel_m1)
        #
        if (self.pos_settings['umbral'] > 0.01):
            self.final_out_m1 = self.out_pos_m1 * (1 - min(abs(self.des_m1) / self.pos_settings['umbral'], 1)) + self.out_vel_m1 * (min(abs(self.des_m1) / self.pos_settings['umbral'], 1))
        else:
            self.final_out_m1 = self.out_vel_m1
        #
        self.limited_out_m1 = self.limit_m1.compute(self.final_out_m1)
        #
        self.m1.publish(self.limited_out_m1)
        #
        #
        self.out_pos_m2 = self.pid_pos_m2.compute(self.angle_lock_m2, self.angle_m2, self.speed_m2)
        self.out_vel_m2 = self.pid_vel_m2.compute(self.des_m2, self.speed_m2, self.accel_m2)
        #
        if (self.times < 10):
            # inicializacion              
            self.angle_lock_m2 = self.angle_m2
        if (self.des_m2 > self.pos_settings['umbral']):
            # no estamos manteniendo ningun angulo
            self.angle_lock_m2 = self.angle_m2
        else:
            # deseamos mantener el angulo
            pass
        #
        if (self.pos_settings['umbral'] > 0.01):
            self.final_out_m2 = self.out_pos_m2 * (1 - min(abs(self.des_m2) / self.pos_settings['umbral'], 1)) + self.out_vel_m2 * (min(abs(self.des_m2) / self.pos_settings['umbral'], 1))
        else:
            self.final_out_m2 = self.out_vel_m2
        #
        self.limited_out_m2 = self.limit_m2.compute(self.final_out_m2)
        #
        self.m2.publish(self.limited_out_m2)
        #
        #
    def spin(self):
        #
        r = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            self.update()
            r.sleep()
            self.times = self.times + 1
            if (self.times % 10 == 0):
                #
                rospy.loginfo("ENC1: e=" + str(self.value_m1) + " s=" + str(self.angle_m1) + " v=" + str(self.speed_m1) + " a=" + str(self.accel_m1))
                rospy.loginfo("ENC2: e=" + str(self.value_m2) + " s=" + str(self.angle_m2) + " v=" + str(self.speed_m2) + " a=" + str(self.accel_m2))
                rospy.loginfo("DESs: vf=" + str(self.vf_val) + " vs=" + str(self.vs_val) + " des_m1=" + str(self.des_m1) + " des_m2=" + str(self.des_m2))
                rospy.loginfo("PID1: outpos=" + str(self.out_pos_m1) + " outvel=" + str(self.out_vel_m1) + " final=" + str(self.final_out_m1) + " lim=" + str(self.limited_out_m1))
                rospy.loginfo("POS1: kp=" + str(self.pid_pos_m1.kp) + " kisum=" + str(self.pid_pos_m1.kisum) + " err=" + str(self.pid_pos_m1.error) + " lck=" + str(self.angle_lock_m1))
                rospy.loginfo("VEL1: kp=" + str(self.pid_vel_m1.kp) + " kisum=" + str(self.pid_vel_m1.kisum) + " err=" + str(self.pid_vel_m1.error))
                rospy.loginfo("PID2: outpos=" + str(self.out_pos_m2) + " outvel=" + str(self.out_vel_m2) + " final=" + str(self.final_out_m2) + " lim=" + str(self.limited_out_m2))
                rospy.loginfo("POS2: kp=" + str(self.pid_pos_m2.kp) + " kisum=" + str(self.pid_pos_m2.kisum) + " err=" + str(self.pid_pos_m2.error) + " lck=" + str(self.angle_lock_m2))
                rospy.loginfo("VEL2: kp=" + str(self.pid_vel_m2.kp) + " kisum=" + str(self.pid_vel_m2.kisum) + " err=" + str(self.pid_vel_m2.error))                
            #
            #
if __name__ == '__main__':
    #
    """ main """
    differential = Differential()
    differential.spin() 
