#!/usr/bin/env python
# -*- coding: utf8 -*-
#
################################################################################
from math import *
################################################################################
import rospy
from std_msgs.msg import Bool
from std_msgs.msg import Int16
from std_msgs.msg import Float32
from dynamic_reconfigure.server import Server
################################################################################
from volti.cfg import VOLConfig
################################################################################
from ino_mod import *
from kfilter import *
from pid_pos import *
from pid_vel import *
from encoder import *
from profile import *
################################################################################
from numpy import *
from numpy.linalg import inv 
################################################################################
from volti.msg import float32_3
from volti.msg import float32_12
from volti.msg import pid_ext
################################################################################
#
class Control:
    #
    def __init__(self, node_name_default = 'control'):
        #
        self.GRLinit(node_name_default)
        self.SRVinit()
        #
        self.pid_vel_m1          = PID_vel(self.vel_settings)
        self.pid_vel_m2          = PID_vel(self.vel_settings)
        #
        #self.pid_vel_vel         = PID_vel(self.vel_settings)
        #self.pid_pos_ang         = PID_pos(self.pos_settings)
        #
        self.integral_ang        = 0
        self.integral_ang_int    = 0
        self.integral_vel        = 0
        #
        # filtro de entradas del encoder
        self.kf_settings         = {'Q' : 10, 'R' : 10, 'P0' : 10, 'rate' : self.rate}
        #
        # filtros kalman de los encoders
        self.filter_e1           = Kfilter(self.kf_settings)
        self.filter_e2           = Kfilter(self.kf_settings)
        self.filter_plate        = Kfilter(self.kf_settings)
        self.filter_pendu        = Kfilter(self.kf_settings)
        #
        # objeto de lectura del encoder
        self.enc_settings = {'offset' : -1}
        #
        self.enc_1 = Encoder(self.enc_settings)
        self.enc_2 = Encoder(self.enc_settings)
        #
        # Objeto de limitacion de la salida a los motores
        self.profile_settings = {'max_output' : 30, 'max_speed' : 10000, 'rate' : self.rate, 'heal_time_at_0pc' : 20, 'stable_point_pc' : 20}
        #
        self.profile_m1 = Profile(self.profile_settings)
        self.profile_m2 = Profile(self.profile_settings)
        #
        self.value_m1 = 0
        self.angle_m1 = 0
        self.speed_m1 = 0
        self.accel_m1 = 0
        #
        self.value_m2 = 0
        self.angle_m2 = 0
        self.speed_m2 = 0
        self.accel_m2 = 0
        #
        self.vel_del_des = 0
        self.ang_lat_des = 0
        #
        self.rollPlate  = 0
        self.pitchPlate = 0
        self.rollPendu  = 0
        self.pitchPendu = 0
        #
        self.con_mode = 0
        #
        self.powsub = rospy.Subscriber("con_mode", Int16, self.modecb)
        #
        # Asociaciones con publicadores y suscriptores
        self.e1_proc_msg = float32_3()
        self.m1    = rospy.Publisher("m1",     Int16)        
        self.e1pub = rospy.Publisher("e1_proc", float32_3)     
        #
        self.e2_proc_msg = float32_3()
        self.m2    = rospy.Publisher("m2",     Int16)            
        self.e2pub = rospy.Publisher("e2_proc", float32_3)     
        #
        self.normalizedkp0rps = 1
        self.normalizedkp1rps = 1
        self.normalizedkpvel  = 1
        self.normalizedkmvel  = 1
        #
        self.rplate_proc_msg = float32_3()
        self.rpendu_proc_msg = float32_3()
        self.rollPenduPub = rospy.Publisher("rpendu", float32_3)
        self.rollPlatePub = rospy.Publisher("rplate", float32_3)
        #
        self.pidangmsg    = pid_ext()
        self.pidvelm1msg  = pid_ext()
        self.pidvelm2msg  = pid_ext()
        self.pidangpub    = rospy.Publisher("pidang", pid_ext)
        self.pidvelm1vel  = rospy.Publisher("pidvm1", pid_ext)
        self.pidvelm2vel  = rospy.Publisher("pidvm2", pid_ext)
        #
        self.err_proc_msg = float32_3()
        self.errPub       = rospy.Publisher("errpro", float32_3)
        self.avg_vel_pub  = rospy.Publisher("avgvel", Float32)
        self.exp_pub      = rospy.Publisher("expdec", Float32)
        #
        self.e1           = rospy.Subscriber("e1",    Int16, self.e1cb)  # entrada del encoder 1
        self.e2           = rospy.Subscriber("e2",    Int16, self.e2cb)  # entrada del encoder 1
        #
        self.veldeldessub = rospy.Subscriber("vel_delante_des", Float32, self.veldeldescb)
        self.anglatdessub = rospy.Subscriber("ang_lateral_des", Float32, self.anglatdescb)
        #
        self.subImuPlate  = rospy.Subscriber('imu_plate_3', float32_3, self.imuplatecb)
        self.subImuPendu  = rospy.Subscriber('imu_pendu_3', float32_3, self.imupenducb)
        #
        self.srv = Server(VOLConfig, self.SRVcallback)
        #
    def modecb(self, data):
        #
        self.con_mode = data.data
        #
    def veldeldescb(self, data):
        #
        self.vel_del_des = data.data
        #
    def anglatdescb(self, data):
        #
        self.ang_lat_des = data.data
        #
    def imuplatecb(self, data):
        #
        self.rollPlate  = data.data[0] - 0.06
        self.pitchPlate = data.data[1]
        #
    def imupenducb(self, data):
        #
        #self.rollPendu  = self.angleRange(data.data[0] + 3.1416)
        if (data.data[0] < 0):
            self.rollPendu = data.data[0] + 3.1416
        if (data.data[0] > 0):
            self.rollPendu = data.data[0] - 3.1416
        self.rollPendu = self.rollPendu + 0.05
        self.pitchPendu = data.data[1]
        #
    def GRLinit(self, node_name_default):
        #
        rospy.init_node(node_name_default)
        self.nodename = rospy.get_name()
        rospy.loginfo("Started node %s", self.nodename)
        #
        self.rate = float(rospy.get_param("rate", 10))
        self.times = 0
        #
    def SRVinit(self):
        #
        # pid_out da el valor de salida al motor de -100 a 100
        # pid_ang da el valor del angulo del encoder en su marco de referencia (lafrom std_msgs.msg import Bool IMU da el angulo de la esfera, en general, dado
        # que cada motor puede dar una cantidad potencialmente infinita de vueltas, no es posible hacer una correspondencia uno a uno
        # con el angulo real del motor. Los encoders se consideran digitales de 0 a 1023 en una vuelta completa
        # pid_vel da el valor de la velocidad del angulo del encoder en su marco de referencia, se calcula en base al valor anterior
        #
        self.pos_settings = {'kp0rps' : 0, 'kp1rps' : 0, 'ki' : 0, 'kd' : 0, 'ka' : 0, 'ks' : 0, 'umbral' : 0, 'umbral_int' : 0, 'umbral_oof' : 0, 'div_minimal' : 1, 'div_ang2vel' : 1, 'div_modes' : 0, 'range' : 0, 'rate' : self.rate}
        self.vel_settings = {'kp' : 0, 'ki' : 0, 'kd' : 0, 'km' : 0, 'ka' : 0, 'ks' : 0, 'umbral' : 0, 'ki_dec' : 0, 'range' : 0, 'rate' : self.rate}
        #
    def SRVcallback(self, config, level):
        #
        rospy.loginfo("reconfiguring")
        #
        #values catched from mouse event from rqt_gui
        self.normalizedkp0rps = float(config['kp0rps_pos'])      
        self.normalizedkp1rps = float(config['kp1rps_pos']) 
        self.normalizedkpvel  = float(config['kp_vel'])
        self.normalizedkmvel  = float(config['km_vel']) 
        #
        self.pos_settings['kp0rps'] = float(config['kp0rps_pos'])      
        self.pos_settings['kp1rps'] = float(config['kp1rps_pos'])      
        self.pos_settings['ki']     = float(config['ki_pos'])           
        self.pos_settings['kd']     = float(config['kd_pos'])
        self.pos_settings['ka']     = float(config['ka_pos'])           
        self.pos_settings['ks']     = float(config['ks_pos'])              
        # variables de limitacion adicionales
        self.pos_settings['umbral']     = float(config['umbral_pos'])
        self.pos_settings['umbral_int'] = float(config['umbral_int'])
        self.pos_settings['umbral_oof'] = float(config['umbral_oof'])
        #
        self.pos_settings['div_minimal'] = float(config['div_minimal'])
        self.pos_settings['div_ang2vel'] = float(config['div_ang2vel'])
        #self.pos_settings['div_modes']   = float(config['div_modes'])
        #
        self.pos_settings['range']      = float(config['range_pos'])
        # parametros de posicion, hacer parametros posteriormente, usar dynamic
        self.vel_settings['kp']     = float(config['kp_vel'])          
        self.vel_settings['ki']     = float(config['ki_vel'])          
        self.vel_settings['kd']     = float(config['kd_vel'])          
        self.vel_settings['km']     = float(config['km_vel']) 
        self.vel_settings['ka']     = float(config['ka_vel'])          
        self.vel_settings['ks']     = float(config['ks_vel'])             
        # variables de limitacion adicionales
        self.vel_settings['umbral'] = float(config['umbral_vel'])
        self.vel_settings['range']  = float(config['range_vel'])
        #
        #self.pid_pos_ang.resetting(self.pos_settings)
        #self.pid_vel_vel.resetting(self.vel_settings)
        self.pid_vel_m1.resetting(self.vel_settings)
        self.pid_vel_m2.resetting(self.vel_settings)
        #
        self.kf_settings['P0']   = float(config['P0'])
        self.kf_settings['Q']    = float(config['Q'])
        self.kf_settings['R']    = float(config['R'])
        self.kf_settings['rate'] = float(config['rate'])
        #
        self.filter_e1.resetting(self.kf_settings)
        self.filter_e1.resetting(self.kf_settings)
        self.filter_plate.resetting(self.kf_settings)
        self.filter_pendu.resetting(self.kf_settings)
        #
        return config
        #
    def e1cb(self, data):
        #
        self.value_m1 = data.data
        self.proc_m1  = self.enc_1.compute(self.value_m1)
        self.X_m1     = self.filter_e1.compute(self.proc_m1)
        #
        self.angle_m1 = self.X_m1[0, 0]
        self.speed_m1 = self.X_m1[1, 0]
        self.accel_m1 = self.X_m1[2, 0]
        #
        self.e1_proc_msg.data[0] = self.angle_m1
        self.e1_proc_msg.data[1] = self.speed_m1
        self.e1_proc_msg.data[2] = self.accel_m1
        #
    def e2cb(self, data):
        #
        self.value_m2 = data.data
        self.proc_m2  = self.enc_2.compute(self.value_m2)
        self.X_m2     = self.filter_e2.compute(self.proc_m2)
        #
        self.angle_m2 = self.X_m2[0, 0]
        self.speed_m2 = self.X_m2[1, 0]
        self.accel_m2 = self.X_m2[2, 0]
        #
        self.e2_proc_msg.data[0] = self.angle_m2
        self.e2_proc_msg.data[1] = self.speed_m2
        self.e2_proc_msg.data[2] = self.accel_m2
        #
    def getKp(self, speed):
        #
        return constrain(self.pos_settings['kp0rps'] - (self.pos_settings['kp0rps'] - self.pos_settings['kp1rps']) * abs(speed), 0, 10000)
        #
    def controller(self):
        #
        self.ang_plate = self.rollPlate
        self.ang_pendu = self.rollPendu
        #
        self.X_plate   = self.filter_plate.compute(self.ang_plate)
        self.X_pendu   = self.filter_pendu.compute(self.ang_pendu)
        #
        self.ang_plate = self.X_plate[0, 0]
        self.ang_pendu = self.X_plate[0, 0]
        #
        self.vel_plate = self.X_plate[1, 0]
        self.vel_pendu = self.X_pendu[1, 0]
        #
        self.ace_plate = self.X_plate[2, 0]
        self.ace_pendu = self.X_pendu[2, 0]
        #
        self.avg_vel_m1_m2     = (self.speed_m1 + self.speed_m2) / 2;
        self.avg_vel_m1_m2_abs = (abs(self.speed_m1) + abs(self.speed_m2)) / 2;
        #
        self.ang_control = self.ang_plate - self.ang_lat_des
        self.vel_control = self.vel_plate
        self.ace_control = self.ace_plate 
        #
        self.decexp = exp(- abs(self.avg_vel_m1_m2) * self.pos_settings['div_minimal'])
        #
        #
        #
        self.rplate_proc_msg.data[0] = self.ang_plate
        self.rplate_proc_msg.data[1] = self.vel_plate
        self.rplate_proc_msg.data[2] = self.ace_plate
        #
        self.rpendu_proc_msg.data[0] = self.ang_pendu
        self.rpendu_proc_msg.data[1] = self.vel_pendu
        self.rpendu_proc_msg.data[2] = self.ace_pendu
        #
        #
        #
        self.err_proc_msg.data[0] = self.ang_control
        self.err_proc_msg.data[1] = self.vel_control
        self.err_proc_msg.data[2] = self.ace_control
        #
        #
        #
        self.exp_pub.publish(self.decexp)
        self.errPub.publish(self.err_proc_msg)
        self.avg_vel_pub.publish(self.avg_vel_m1_m2)
        self.e1pub.publish(self.e1_proc_msg)
        self.e2pub.publish(self.e2_proc_msg)
        self.rollPenduPub.publish(self.rpendu_proc_msg)
        self.rollPlatePub.publish(self.rplate_proc_msg)
        #
        #
        #
        self.fi    = self.ang_plate - self.ang_lat_des
        self.beta  = self.ang_pendu - self.ang_plate
        self.fip   = self.vel_plate
        self.betap = self.vel_pendu - self.vel_plate
        #
        """
        self.u = ((-5*self.fi*(-0.419163868 + 0.16569095399999997*cos(self.beta + \
        self.fi)))/(2*((9 + (10*(-0.419163868 + \
        0.16569095399999997*cos(self.beta + self.fi)))/(0.4278062687504448 + \
        0.032367102026846464*cos(self.fi) - \
        0.013726746118715057*cos(2*(self.beta + self.fi))))/4 - \
        (5*(-0.419163868 + 0.16569095399999997*cos(self.beta + \
        self.fi)))/(2*(0.4278062687504448 + 0.032367102026846464*cos(self.fi) \
        - 0.013726746118715057*cos(2*(self.beta + \
        self.fi)))))*(0.4278062687504448 + 0.032367102026846464*cos(self.fi) \
        - 0.013726746118715057*cos(2*(self.beta + self.fi)))) - (self.fi*(9 + \
        (10*(-0.419163868 + 0.16569095399999997*cos(self.beta + \
        self.fi)))/(0.4278062687504448 + 0.032367102026846464*cos(self.fi) - \
        0.013726746118715057*cos(2*(self.beta + \
        self.fi))))*(0.4278062687504448 + 0.032367102026846464*cos(self.fi) - \
        0.013726746118715057*cos(2*(self.beta + self.fi))))/(10*(-0.419163868 \
        + 0.16569095399999997*cos(self.beta + self.fi))) - \
        (2*self.fip*(0.4278062687504448 + 0.032367102026846464*cos(self.fi) - \
        0.013726746118715057*cos(2*(self.beta + self.fi)))*(3 + \
        (-0.026093055573967 + 0.03236710202684647*self.fip*sin(self.fi) - \
        0.13890332234250014*self.betap*sin(self.beta + self.fi) - \
        0.13890332234250014*self.fip*sin(self.beta + \
        self.fi))/(0.4278062687504448 + 0.032367102026846464*cos(self.fi) - \
        0.013726746118715057*cos(2*(self.beta + self.fi)))))/(5*(-0.419163868 \
        + 0.16569095399999997*cos(self.beta + self.fi))))
        """
        #
        self.u = -(((1.0866596181101764 + 0.0520677785084838*cos(self.fi) - \
        0.04382967488275723*cos(2*(self.beta + self.fi)))*(36*self.fip*(3. + \
        (1.*self.fip*sin(self.fi) + (-7.668477375828778*self.betap - \
        7.668477375828778*self.fip)*sin(self.beta + \
        self.fi))/(20.87009757739364 + 1.*cos(self.fi) - \
        0.8417811579116194*cos(2*(self.beta + self.fi)))) + self.fi*(81 + \
        (3233.405311403718*(2.2774557566878655 - 1.*cos(self.beta + \
        self.fi))**2)/(20.87009757739364 + 1.*cos(self.fi) - \
        0.8417811579116194*cos(2*(self.beta + self.fi)))**2 + \
        36*(((19.65931863727455 + 0.5*self.fip**2)*cos(self.fi) + \
        (-3.834238687914389*self.betap**2 - \
        7.668477375828778*self.betap*self.fip - \
        3.834238687914389*self.fip**2)*cos(self.beta + self.fi) - \
        66.1953760249538*cos(2*(self.beta + self.fi)) + \
        7.541694904952274*self.betap*sin(self.beta + \
        self.fi))/(20.87009757739364 + 1.*cos(self.fi) - \
        0.8417811579116194*cos(2*(self.beta + self.fi))) - ((0. + \
        0.894309732065766*self.betap - \
        0.3926792998896158*self.betap*cos(self.beta + self.fi) + \
        (1.023617048433319 + 0.026033889254241903*self.fip**2)*sin(self.fi) + \
        (-0.19964029075098597*self.betap**2 - \
        0.39928058150197193*self.betap*self.fip - \
        0.19964029075098597*self.fip**2)*sin(self.beta + self.fi) - \
        1.7233230885765467*sin(2*(self.beta + \
        self.fi)))*(-0.0520677785084838*sin(self.fi) + \
        0.08765934976551445*sin(2*(self.beta + \
        self.fi))))/(1.0866596181101764 + 0.0520677785084838*cos(self.fi) - \
        0.04382967488275723*cos(2*(self.beta + \
        self.fi)))**2))))/(-60.686428700700006 + 26.646589521*cos(self.beta + \
        self.fi))) -2.5 * self.beta
        #
        """
        u esta dada en terminos de voltaje, pero necesitamos pasarla a un porcentaje que oueda ir de 0
        a 100, como en el resto del programa, y dejar que las funciones "administren" la zona muerta, la conversion
        a porcenta podria ser algo como

        pc = u * 100 / 15

        para un maximo asumido de 15 volts, con un sensor este valor podria cambiar de manera dinamica
        """
        #
        #
        self.sal_u_pc = self.u * 100 / 13.6
        self.sal_u_pc_fix = sign(self.sal_u_pc) * 5 * self.decexp + self.sal_u_pc
        #
        #
        if abs(self.ang_control) < self.pos_settings['umbral']:
            self.ang_control_tmp = 0
        else:
            self.ang_control_tmp = self.ang_control
        # 
        #
        self.pidangmsg.kp = self.getKp(abs(self.avg_vel_m1_m2)) * self.ang_control_tmp
        self.pidangmsg.kd = self.pos_settings['kd'] * self.vel_control
        self.pidangmsg.ka = self.pos_settings['ka'] * self.ace_control
        self.pidangmsg.ks = self.pos_settings['ks'] * sin(self.ang_control)
        #
        self.salida_m1_ang = self.pidangmsg.kp + self.pidangmsg.kd + self.pidangmsg.ka - self.pidangmsg.ks
        #
        self.integral_ang  = constrain(self.integral_ang + self.ang_control / self.rate, -1, 1)
        if abs(self.ang_control) < self.pos_settings['umbral_int']:
             self.integral_ang = 0
        #
        self.integral_ang_int  = constrain(self.integral_ang_int + self.ang_control / self.rate, -1, 1)
        if abs(self.ang_control) > self.pos_settings['umbral_int']:
            self.integral_ang_int = 0
        #
        self.pidangmsg.ki = 3 * self.integral_ang_int * self.decexp
        #
        self.salida_m1_ang = self.salida_m1_ang + self.pidangmsg.ki
        #
        if abs(self.ang_control) < self.pos_settings['umbral_oof']:
            self.salida_m1_ang = sign(self.salida_m1_ang) * 5 * self.decexp
        else:
            self.salida_m1_ang = sign(self.salida_m1_ang) * 5 * self.decexp + self.salida_m1_ang
        #
        self.salida_m1_ang = constrain(self.salida_m1_ang, -self.pos_settings['range'], self.pos_settings['range'])
        self.salida_m1_ang = self.salida_m1_ang + self.pos_settings['ki'] * self.integral_ang
        self.pidangmsg.out = self.salida_m1_ang
        #
        #
        self.salida_m1_ang = constrain(self.sal_u_pc_fix  + self.pidangmsg.ki, -self.pos_settings['range'], self.pos_settings['range'])
        self.pidangmsg.out = self.salida_m1_ang
        #
        #
        self.vel_m1_des = self.vel_del_des + self.ang_control / self.pos_settings['div_ang2vel']
        self.vel_m2_des = self.vel_del_des - self.ang_control / self.pos_settings['div_ang2vel']
        #
        self.vel_m1_err = self.vel_m1_des - self.speed_m1
        self.vel_m2_err = self.vel_m2_des - self.speed_m2
        #
        #
        #
        self.pidvelm1msg.kp  = self.vel_settings['kp'] * self.vel_m1_err
        self.pidvelm1msg.kd  = self.vel_settings['kd'] * self.accel_m1 
        self.pidvelm1msg.km  = self.vel_settings['km'] * self.vel_m1_des
        self.pidvelm1msg.sub = (self.vel_settings['kp'] + self.vel_settings['km']) * self.ang_control / self.pos_settings['div_ang2vel']
        #
        self.pidvelm2msg.kp  = self.vel_settings['kp'] * self.vel_m2_err
        self.pidvelm2msg.kd  = self.vel_settings['kd'] * self.accel_m2 
        self.pidvelm2msg.km  = self.vel_settings['km'] * self.vel_m2_des
        self.pidvelm2msg.sub = (self.vel_settings['kp'] + self.vel_settings['km']) * -self.ang_control / self.pos_settings['div_ang2vel']
        #
        #
        #
        self.salida_m1_vel = self.pidvelm1msg.kp - self.pidvelm1msg.kd + self.pidvelm1msg.km
        self.salida_m2_vel = self.pidvelm2msg.kp - self.pidvelm2msg.kd + self.pidvelm2msg.km
        #
        self.salida_m1_vel = self.salida_m1_vel + (self.vel_settings['kp'] + self.vel_settings['km']) * self.ang_control / self.pos_settings['div_ang2vel']
        self.salida_m2_vel = self.salida_m2_vel + (self.vel_settings['kp'] + self.vel_settings['km']) * -self.ang_control / self.pos_settings['div_ang2vel']
        #
        self.salida_m1_vel = constrain(self.salida_m1_vel, -self.vel_settings['range'], self.vel_settings['range'])
        self.salida_m2_vel = constrain(self.salida_m2_vel, -self.vel_settings['range'], self.vel_settings['range'])
        #
        self.out_pos_m1 = self.profile_m1.compute( self.salida_m1_ang * self.decexp + self.salida_m1_vel )
        self.out_pos_m2 = self.profile_m2.compute( -self.salida_m1_ang * self.decexp + self.salida_m2_vel )
        #
        #
        self.pidvelm1msg.out = self.salida_m1_vel 
        self.pidvelm2msg.out = self.salida_m2_vel 
        #
        #
        self.pidvelm1vel.publish(self.pidvelm1msg)
        self.pidvelm2vel.publish(self.pidvelm2msg)
        #
        #
        if (self.con_mode is 1):
            return
        #
        self.m1.publish(int((self.out_pos_m1) * 100))
        self.m2.publish(int((self.out_pos_m2) * 100))
        #
    def update(self):
        #
        self.times = self.times + 1
        #
        self.controller()
        #
    def spin(self):
        #
        r = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            self.update()
            r.sleep()
            #
if __name__ == '__main__':
    #
    """ main """
    control = Control()
    control.spin()
    #