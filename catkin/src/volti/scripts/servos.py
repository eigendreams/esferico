#!/usr/bin/env python
# -*- coding: utf8 -*-

import rospy
import time
usleep = lambda x: time.sleep(x/1000000.0)
from std_msgs.msg import Int16
import Adafruit_BBIO.PWM as PWM
from ino_mod import *
import atexit

class Servos:
    #
    def __init__(self, node_name_override = 'servos'):
        #
        # El nombre del node deberìa poder especificarse como override desde alguna
        # instancia superior, por si acaso, se establece un default razonable.
        # De todos modos, el nombre del nodo que se guarda "internamente" no puede
        # asumirse igual al proporcionado, por si se invocasen al mismo tiempo varios
        # nodos del mismo "tipo", pues ROS les anexa un número al final como ID
        rospy.init_node(node_name_override)
        self.nodename = rospy.get_name()
        # loginfo sirve como medio de DEBUGGING, y para guardar un registro de la
        # ejecución del programa. Por ejemplo, algón overnodo podría hacer uso del log
        # para arreglar errores
        rospy.loginfo("Node starting with name %s", self.nodename) 
        # Quizá algún parámetro superior especifique una frecuencia de salida de datos,
        # por default, si no se encuentra, se usa una de 1 Hz 
        self.rate = float(rospy.get_param("rate", 10))
        #
        self.m1pin = "P9_22"
        self.m2pin = "P9_21"
        #
        self.m1val = 0
        self.m2val = 0
        self.alval = 0
        #
        self.inittime = rospy.get_time()
        self.timelastal = -1000
        #
        self.m1sub = rospy.Subscriber("m1", Int16, self.m1cb)
        self.m2sub = rospy.Subscriber("m2", Int16, self.m2cb)
        self.alsub = rospy.Subscriber("al", Int16, self.alcb)
        #
    def m1cb(self,data):
        #
        self.m1val = data.data
        #
    def m2cb(self, data):
        #
        self.m2val = data.data
        #
    def alcb(self, data):
        #
        self.timelastal = millis(self.inittime)
        self.alval = data.data
        #
    def getpercent(self, data):
        #
        self.center = 1500. / 200.
        self.remain = 2.5 * data / 100. 
        return (self.center + self.remain)
        #
    def update(self):
        #
        if ((millis(self.inittime) - self.timelastal) > 1500):
            PWM.start(self.m1pin, self.getpercent(7.5, 50)
            PWM.start(self.m2pin, self.getpercent(7.5, 50)
        else:
            PWM.start(self.m1pin, self.getpercent(self.m1val / 100.), 50)
            PWM.start(self.m2pin, self.getpercent(self.m2val / 100.), 50)
        #
    def spin(self):
        #
        # rospy.Rate y sleep permiten que el nodo pare su ejecución (cuidado, los subscriptores
        # NO están restringidos por el resto del nodo más allá de la función de callback) e 
        # intente ejecutarse cada tantos ms, dados por el rate declarado en el constructor de la
        # clase. Ahorra recursos
        r = rospy.Rate(self.rate)
        #
        while not rospy.is_shutdown():
            self.update()
            r.sleep()
        #
# Invocación del constructor. Básicamente, si este archivo fuera a ejecutarse como "top level", el
# intérprete le asignaría el "nombre" (protegido) __main__ y ejecutaría las siguientes lineas como
# LO PRIMERO en el archivo (salvo imports y declaraciones) 

def onclose():
    PWM.start("P9_22", 7.5, 50)
    PWM.start("P9_21", 7.5, 50)

if __name__ == '__main__':
    """ main """
    atexit.register(onclose())
    servos = Servos()
    servos.spin()
    