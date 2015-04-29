#!/usr/bin/env python

import rospy

from dynamic_reconfigure.server import Server
from volti.cfg import PIDvelConfig

def callback(config, level):
    rospy.loginfo("""Reconfiugre Request: {kp_vel}, {ki_vel}, {kd_vel}, {km_vel}, {umbral_vel}, {range_vel}, {kierr_vel}, {kimax_vel}""".format(**config))
    return config

if __name__ == "__main__":
    rospy.init_node("volti", anonymous = True)
    srv = Server(PIDvelConfig, callback)
    rospy.spin()