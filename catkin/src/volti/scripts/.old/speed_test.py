#!/usr/bin/env python
# -*- coding: utf8 -*-

import rospy
from std_msgs.msg import Int16

class Interface_Servo:
    
    def __init__(self, node_name_override = 'interface_servo'):
        
        rospy.init_node(node_name_override)
        self.nodename = rospy.get_name()
        rospy.loginfo("interface_servo starting with name %s", self.nodename) 
        self.freq = 10
        self.rate = rospy.get_param("interface_servo_rate", self.freq)

        self.Mot1Pub = rospy.Publisher("mot1", Int16)
        self.Mot2Pub = rospy.Publisher("mot2", Int16)

        self.time_lock = 0
        self.time_max  = 20000   # valor en ms
        self.servo_val = 1000

        self.state = 0

        self.StopSub  = rospy.Subscriber("stop" , Int16, self.stopcb)
        self.StartSub = rospy.Subscriber("start", Int16, self.startcb)


    def stopcb(self, data):

        self.state = 0

        self.servo_val = 1000
        self.Mot1Pub.publish(self.servo_val)
        self.Mot2Pub.publish(self.servo_val)

    def startcb(self, data):

        self.state = 1

        self.servo_val = 1000
        self.Mot1Pub.publish(self.servo_val)
        self.Mot2Pub.publish(self.servo_val)

    def bolitaCb(self, data):
        if (data.data > 0):
            if (self.time_lock > 0):
                # Estamos DENTRO de una operacion de movimiento de bolita!!!
                # debemos esperar a que termine el proceso y no hacer nada!!!
                return 0

            # reiniciando el contador del proceso al time max
            self.time_lock = self.time_max

    def constrain(self, x, min, max):

        if (x > max):
            return max
        if (x < min):
            return min
        return x
        
    def update(self):

        # decrementando el valor de tiempo transcurrido del proceso
        if (self.time_lock > 0):
            self.time_lock = self.time_lock - 1000. / self.freq
        else:
            self.time_lock = 0

        # calculando el valor de angulo deseado para el servo de la bolita
        if (self.state > 0):

            self.servo_val = self.constrain(self.servo_val + 2, 1000, 1250);

            self.Mot1Pub.publish(self.servo_val)
            self.Mot2Pub.publish(self.servo_val)
        
    def spin(self):
        
        # rospy.Rate y sleep permiten que el nodo pare su ejecución (cuidado, los subscriptores
        # NO están restringidos por el resto del nodo más allá de la función de callback) e 
        # intente ejecutarse cada tantos ms, dados por el rate declarado en el constructor de la
        # clase. Ahorra recursos
        r = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            self.update()
            r.sleep()

# Invocación del constructor. Básicamente, si este archivo fuera a ejecutarse como "top level", el
# intérprete le asignaría el "nombre" (protegido) __main__ y ejecutaría las siguientes lineas como
# LO PRIMERO en el archivo (salvo imports y declaraciones)    
if __name__ == '__main__':
    """ main """
    interface_servo = Interface_Servo()
    interface_servo.spin()
    

