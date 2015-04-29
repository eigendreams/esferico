#!/usr/bin/env python
# -*- coding: utf8 -*-

import rospy
from std_msgs.msg import Int16

class Alive:
    #
    def __init__(self, node_name_override = 'alive'):
        #
        rospy.init_node(node_name_override)
        self.nodename = rospy.get_name()
        rospy.loginfo("Node starting with name %s", self.nodename) 
        #
        # el codigo de la stellaris esta dispuesto de manera que si se pierde comunicacion con el BBB por
        # mas de medio segundo, los motores deben apagarse, esto puede suceder porque
        #   algun programa en el BBB trabo el sistema
        #   se apago el BBB
        # Aqui el valor de alive se emite cada segundo, de manera que, luego de 1s de timeout
        # si no se recibe alive=1, el nodo de push_servos enviara un cero como alive y se apagaran los motores
        # esto puede suceder porque
        #   el usuario decide apagar los motores con el control, haciendo alive=0 directamente
        #   se perdio contacto por wifi
        # este nodo esta pensado para ser usado con la terminal para debugeo, si se usa el nodo de control, ese mismo
        # implementa una version de este nodo
        #
        # Este nodo SIEMPRE debe ejecutarse en el equipo de tierra
        #
        self.rate = rospy.get_param("alive_rate", 1)
        #
        # Eventualmente, el valor de alive podria reflejar diferente modos de operacion sobre sus bits
        #
        self.alive_val = 1
        self.alivePub = rospy.Publisher('al', Int16)
        #
    def update(self):
        #
        self.alivePub.publish(self.alive_val)
        #
    def spin(self):
		#
        r = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            self.update()
            r.sleep()
        #
if __name__ == '__main__':
    """ main """
    alive = Alive()
    alive.spin()
    