#!/usr/bin/env python
# -*- coding: utf8 -*-
#
################################################################################
import rospy
from   std_msgs.msg import Bool
from   std_msgs.msg import Int16
from   std_msgs.msg import Float32
from   std_msgs.msg import String
from   volti.msg    import float32_3
from   volti.msg    import float32_12
import time
import atexit
################################################################################
#
class Logger:
    #
    def __init__(self, node_name_default = 'logger'):
        #
        self.f = open(str(time.strftime("%y%m%d%H%M%S")) + ".csv", 'w')
        atexit.register(self.onclose)
        #
        rospy.init_node(node_name_default)
        self.nodename = rospy.get_name()
        rospy.loginfo("Node starting with name %s", self.nodename)
        self.rate = int(rospy.get_param("rate", '100'))
        #
        self.times = 0
        #
        self.alertval = ""
        self.anglatdesval = 0.
        self.avgvelval = 0.
        self.e1val = 0
        self.e2val = 0
        self.e1pval = float32_3()
        self.e2pval = float32_3()
        self.erpval = float32_3()
        self.decval = 0.
        self.ipendval = float32_3()
        self.iplatval = float32_3()
        self.m1val = 0
        self.m2val = 0
        self.rpendval = float32_3()
        self.rplatval = float32_3()
        self.velveldesval = 0.
        #
        self.s1  = rospy.Subscriber("alert",           String,    self.alertcb)
        self.s2  = rospy.Subscriber("ang_lateral_des", Float32,   self.anglatdescb)
        self.s3  = rospy.Subscriber("avgvel",          Float32,   self.avgvelcb)
        self.s4  = rospy.Subscriber("e1",              Int16,     self.e1cb)
        self.s5  = rospy.Subscriber("e2",              Int16,     self.e2cb)
        self.s6  = rospy.Subscriber("e1_proc",         float32_3, self.e1pcb)
        self.s7  = rospy.Subscriber("e2_proc",         float32_3, self.e2pcb)
        self.s8  = rospy.Subscriber("errpro",          float32_3, self.erpcb)
        self.s9  = rospy.Subscriber("expdec",          Float32,   self.deccb)
        self.s10 = rospy.Subscriber("imu_pendu_3",     float32_3, self.ipendcb)
        self.s11 = rospy.Subscriber("imu_plate_3",     float32_3, self.iplatcb)
        self.s12 = rospy.Subscriber("m1",              Int16,     self.m1cb)
        self.s13 = rospy.Subscriber("m2",              Int16,     self.m2cb)
        self.s14 = rospy.Subscriber("rpendu",          float32_3, self.rpendcb)
        self.s15 = rospy.Subscriber("rplate",          float32_3, self.rplatcb)
        self.s16 = rospy.Subscriber("vel_delante_des", Float32,   self.velveldescb)
        #
    def alertcb(self, data):
        self.alertval = data.data
    def anglatdescb(self, data):
        self.anglatdesval = data.data
    def avgvelcb(self, data):
        self.avgvelval = data.data
    def e1cb(self, data):
        self.e1val = data.data
    def e2cb(self, data):
        self.e2val = data.data
    def e1pcb(self, data):
        self.e1pval = data
    def e2pcb(self, data):
        self.e2pval = data
    def erpcb(self, data):
        self.erpval = data
    def deccb(self, data):
        self.decval = data.data
    def ipendcb(self, data):
        self.ipendval = data
    def iplatcb(self, data):
        self.iplatval = data
    def m1cb(self, data):
        self.m1val = data.data
    def m2cb(self, data):
        self.m2val = data.data
    def rpendcb(self, data):
        self.rpendval = data
    def rplatcb(self, data):
        self.rplatval = data
    def velveldescb(self, data):
        self.velveldesval = data.data
    def update(self):
        #
        self.times = self.times + 1
        self.f.write(str(self.times / 10) + "." + str(((self.times % 10) * 1000)/10).zfill(3)  + "," + \
        str(self.alertval).replace(',', ' ') + "," + \
        str(self.anglatdesval) + "," + \
        str(self.avgvelval) + "," + \
        str(self.e1val) + "," + \
        str(self.e2val) + "," + \
        str(self.e1pval.data[0]) + "," + \
        str(self.e1pval.data[1]) + "," + \
        str(self.e1pval.data[2]) + "," + \
        str(self.e2pval.data[0]) + "," + \
        str(self.e2pval.data[1]) + "," + \
        str(self.e2pval.data[2]) + "," + \
        str(self.erpval.data[0]) + "," + \
        str(self.erpval.data[1]) + "," + \
        str(self.erpval.data[2]) + "," + \
        str(self.decval) + "," + \
        str(self.ipendval.data[0]) + "," + \
        str(self.iplatval.data[0]) + "," + \
        str(self.m1val) + "," + \
        str(self.m2val) + "," + \
        str(self.rpendval.data[0]) + "," + \
        str(self.rpendval.data[1]) + "," + \
        str(self.rpendval.data[2]) + "," + \
        str(self.rplatval.data[0]) + "," + \
        str(self.rplatval.data[1]) + "," + \
        str(self.rplatval.data[2]) + "," + \
        str(self.velveldesval) + "," + \
        '\n')               
        #
        #
    def onclose(self):
        #
        self.f.close()
        #
    def spin(self):
        #
        self.f.write("time , alert , anglatdes , avgvel , e1 , e2 , e1k , e1pk , e1ppk , e2k , e2pk , e2ppk , errk , errpk , errppk , dec , ipenduroll , iplateroll , m1 , m2 , rpenduk , rpendupk , rpenduppk ,rplatek , rplatepk , rplateppk , veldeldes" + '\n')
        #
        r = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            self.update()
            r.sleep()
            #
            #
if __name__ == '__main__':
    #
    """ main """
    logger = Logger()
    logger.spin() 
