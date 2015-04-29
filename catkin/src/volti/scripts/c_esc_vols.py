#!/usr/bin/env python
# -*- coding: utf8 -*-

import rospy
from std_msgs.msg import Int16
from ino_mod import *

class Control_ESC_Volantes:
    #
    def __init__(self, node_name_override = 'c_esc_vols'):
        #
        rospy.init_node(node_name_override)
        self.nodename = rospy.get_name()
        rospy.loginfo("Node starting with name %s", self.nodename) 
        #
        self.rate = rospy.get_param("rate", 10)
        #
        # state = -1 -> volantes apagandose
        # state  = 0 -> volantes apagados
        # state  = 1 -> volantes encendiendo
        # state  = 2 -> volante a velocidad crucero
        #
        # vol1 y vol2 de 0 a 100,00
        # 
        self.state     = 0
        #
        self.init_time = rospy.get_time()
        self.tick_time = 1. / self.rate
        self.task_done = False
        #
        self.signal    = 0
        #
        self.vol1      = 0
        self.vol2      = 0
        #
        self.rise_time = int(rospy.get_param("rise_vol_segs", 30))
        #
        # Los datos de salida a los ESC deberan seguir el patron del resto del codigo, con una diferencia
        # En general, vamos de -100,00 a 100,00, en este caso, solo tiene sentido ir de 0 a 100,00
        # se dejan dos publicadores, por si acaso se quiera implementar alguna funcionalidad en otro nodo que 
        # necesite hacer un uso dispar de los volantes
        #
        self.vol1Pub = rospy.Publisher('vol1', Int16)
        self.vol2Pub = rospy.Publisher('vol2', Int16)
        #
        # Las senales de control seran simplemente
        #
        # 0 apagar los volantes, si una secuencia esta ya activada, el apagado debera de ser gradual
        # 1-100 encender los volantes al porcentaje, si una secuencia no esta activada, el encendido debera ser gradual
        # 
        self.sgnSub  = rospy.Subscriber('vol_sgn', Int16, self.sgncb)
        #
    def sgncb(self, data):
        #
        self.signal = data.data
        #
    def update(self):
        #
        # Este nodo trata de brindar una secuencia de arranque suave para los volantes, por el potencial golpe que podria
        # sufrir el robot, de iniciarse de forma violenta. Se tiene pensado que los volantes alcanzen la velocidad maxima
        # en 30 segundos, como maximo, mediante parametros que se definen como parametros de ROS, pero con defaults
        # especificados en el codigo superior. 
        #
        # Una posibilidad mas seria que la senal de control de los volantes, puesto que deben girar a la misma velocidad, se
        # repita en el micro para ambos, y se ahorren problemas de retraso de comunicaciones que se reflejen en jerk
        #
        # Es una propuesta aceptable, pero deberia brindarse alguna forma de comunicacion explicita de ello fuera del codigo
        # Propongo usar algun mensaje adicional, por ejemplo, usar alive como una serie de codigos de control que especifiquen el
        # modo para varios sistemas en forma de una mascara de bits, con un maximo de 14 opciones, la primera prohibida porque tiene signo,
        # y la ultima porque especifica el estado como alive o no
        #
        # ok, pero no sera implementado aun
        #
        # Esta clase necesita de varios parametros:
        #
        # RISE_TIME o el tiempo necesario para alcanzar la velocidad de crucero
        # CON_SIGN   o una senal que le indique cuando parar o iniciar los volantes, todos los inicios deben de durar ej 30 segundos
        #            pero los paros podrian hacerse simplemente apagando la salida de los ESC, recomiendo, que los paros se hagan de
        #            manera controlada y durante un tiempo total de ej dos minutos, por la tremenda energia que se tiene que disipar
        #
        # Usamos una maquina de estados desacoplada del resto del flujo del programa, porque es necesario asegurar que los estados se
        # cumplan siempre de forma precisa, no hacerlo podria ser peligroso, por la gran velocidad de los volantes, como consecuencia, cada
        # estado debe completarse, antes de poder psara al siguiente
        #
        # reglas sobre el estado
        # operaciones en el estado
        # salidas en los estados
        #
        # transcisiones de estados
        #
        if (self.state > 0 and selg.signal == 0):
            self.state = -1
            self.task_done = False
            self.task_time = 0
        if (self.state == -1 and self.task_done):
            self.state = 0
        if (self.state == 0 and self.signal == 1):
            self.state = 1
            self.task_done = False
            self.task_time = 0
        if (self.state == 1 and self.task_done):
            self.state = 2
        #
        # acciones dentro de los estados
        #
        if (self.state == -1):
            # secuencia de apagado, que dure por lo menos cuatro veces el tiempo de encendido
            if (self.vol1 == 0 and self.vol2 == 0):
                self.task_done = True
            #
            self.vol1 = constrain(self.vol1 - 100 * self.tick_time / ( 4. * self.rise_time), 0, 100)
            self.vol2 = constrain(self.vol2 - 100 * self.tick_time / ( 4. * self.rise_time), 0, 100)
            #
        if (self.state == 0):
            # estado de espera, no hacer nada, quiza mandar a cero los volantes
            self.vol1 = 0
            self.vol2 = 0
            #
        if (self.state == 1):
            # encendiendo
            if (self.vol1 == self.signal and self.vol2 == self.signal):
                self.task_done = True
            #
            self.vol1 = constrain(self.vol1 + self.signal * self.tick_time / (self.rise_time), 0, self.signal)
            self.vol2 = constrain(self.vol2 + self.signal * self.tick_time / (self.rise_time), 0, self.signal)
        if (self.state == 2):
            # encendido, quiza publicar el valor de senal constantemente
            self.vol1 = self.signal
            self.vol2 = self.signal
        #
        # salidas, independientes de los estados pero quiza dependientes de modos de operacion
        #
        # la publicacion se hace de manera continua, no tiene porque ser asi, pero si se agregase algun modo
        # de operacion diferente, por ejemplo, para pruebas manuales, la division de modos de salida se haria aqui pero
        # la maquina de estados y demas seguiria funcionando, aunque no se le de salida
        #
        self.vol1Pub.publish(self.vol1 * 100)
        self.vol2Pub.publish(self.vol2 * 100)
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
    c_esc_vols = Control_ESC_Volantes()
    c_esc_vols.spin()
    