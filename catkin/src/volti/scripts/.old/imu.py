#!/usr/bin/env python

import math
################################################################################
import rospy
from std_msgs.msg import Bool
from std_msgs.msg import Int16
from std_msgs.msg import Float32
################################################################################
from   volti.msg    import float32_12
from   numpy        import *
from   numpy.linalg import inv 
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

class Top_control:
    #
    def __init__(self, node_name_default = 'top_control_node'):
        #
        rospy.init_node(node_name_default)
        self.nodename = rospy.get_name()
        rospy.loginfo("top_control_node starting with name %s", self.nodename)
        self.rate = float(rospy.get_param("param_global_rate", '10'))
        #
        self.times = 0
        #
        self.vf_sphere = 0
        self.vs_sphere = 0
        self.vf_pendulum = 0
        self.vs_pendulum = 0
        #
        self.imu1times = 0
        self.imu2times = 0
        #
        # Asociaciones con publicadores y suscriptores
        self.m1 = rospy.Publisher("vf", Float32)  # salida al motor 1
        self.m2 = rospy.Publisher("vs", Flaot32)  # 
        self.sub1 = rospy.Subscriber('i1', float32_12, self.processIMU1_message)
        self.sub2 = rospy.Subscriber('i2', float32_12, self.processIMU2_message)
        self.subVF = rospy.Subscriber('vf_sphere', Float32, self.vfsphcb)
        self.subVS = rospy.Subscriber('vs_sphere', Float32, self.vssphcb)
        #
    def vfsphcb(self, data):
        #
        self.vf_sphere = data.data
        #
    def vssphcb(self, data):
        #
        self.vs_sphere = data.data
        #
    def processIMU1_message(rawMsg):
        #
        self.imu1times = self.imu1times + 1
        #
        self.yaw = rawMsg.data[0] * 3.1416 / 180
        self.pitch = rawMsg.data[1] * 3.1416 / 180
        self.roll = rawMsg.data[2] * 3.1416 / 180
        #
        self.R_yaw_z = array([[cos(self.yaw), sin(self.yaw), 0], [-sin(self.yaw), cos(self.yaw, 0)], [0, 0, 1]])
        self.R_pitch_y = array([[cos(self.pitch), 0, sin(self.pitch)],[0, 1, 0],[-sin(self.pitch), 0, cos(self.pitch)]])
        self.R_roll_x = array([[1, 0, 0],[0, cos(self.roll), -sin(self.roll)],[0, sin(self.roll), cos(self.roll)]])
        #
        self.x_axis_imu = dot(self.R_yaw_z, dot(self.R_pitch_y, dot(self.R_roll_x, array([[1],[0],[0]]))))
        self.y_axis_imu = dot(self.R_yaw_z, dot(self.R_pitch_y, dot(self.R_roll_x, array([[0],[1],[0]]))))
        self.z_axis_imu = dot(self.R_yaw_z, dot(self.R_pitch_y, dot(self.R_roll_x, array([[0],[0],[1]]))))
        #
    def processIMU2_message(rawMsg):
        #
        self.imu2times = self.imu2times + 1
        #
        self.yaw2 = rawMsg.data[0] * 3.1416 / 180
        self.pitch2 = rawMsg.data[1] * 3.1416 / 180
        self.roll2 = rawMsg.data[2] * 3.1416 / 180
        #
        self.R_yaw_z2 = array([[cos(self.yaw2), sin(self.yaw2), 0], [-sin(self.yaw2), cos(self.yaw2, 0)], [0, 0, 1]])
        self.R_pitch_y2 = array([[cos(self.pitch2), 0, sin(self.pitch2)],[0, 1, 0],[-sin(self.pitch2), 0, cos(self.pitch2)]])
        self.R_roll_x2 = array([[1, 0, 0],[0, cos(self.roll2), -sin(self.roll2)],[0, sin(self.roll2), cos(self.roll2)]])
        #
        self.x_axis_imu2 = dot(R_yaw_z2, dot(R_pitch_y2, dot(R_roll_x2, array([[1],[0],[0]]))))
        self.y_axis_imu2 = dot(R_yaw_z2, dot(R_pitch_y2, dot(R_roll_x2, array([[0],[1],[0]]))))
        self.z_axis_imu2 = dot(R_yaw_z2, dot(R_pitch_y2, dot(R_roll_x2, array([[0],[0],[1]]))))
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



