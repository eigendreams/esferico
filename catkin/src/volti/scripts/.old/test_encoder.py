#!/usr/bin/env python
# -*- coding: utf8 -*-

import Adafruit_BBIO.GPIO as GPIO
import time

usleep = lambda x: time.sleep( x / 1000000.0)

GPIO.cleanup()

"""
csnpin = 'P8_8'
clkpin = 'P8_10'
propin = 'P8_12'
lecpin = 'P8_14'
"""

csnpin = 'P8_7'
clkpin = 'P8_9'
propin = 'P8_11'
lecpin = 'P8_15'

GPIO.setup(csnpin, GPIO.OUT)
GPIO.setup(clkpin, GPIO.OUT)
GPIO.setup(propin, GPIO.OUT)
GPIO.setup(lecpin, GPIO.IN)

maxdevices = 1
base_bits = 16 * maxdevices + maxdevices - 1
chainData = 0
index = 0

def begincom():
	#
	GPIO.output(clkpin, GPIO.LOW)
	GPIO.output(propin, GPIO.LOW)
	GPIO.output(csnpin, GPIO.HIGH)

def readenc():
	#
	chainData = 0
	#
	GPIO.output(csnpin, GPIO.LOW)
	
	GPIO.output(clkpin, GPIO.LOW)
	pass
	#
	for k in range(base_bits):
		GPIO.output(clkpin, GPIO.HIGH)
		pass
		chainData = (chainData << 1) | (GPIO.input(lecpin) & 1)
		GPIO.output(clkpin, GPIO.LOW)
		pass
	#
	GPIO.output(propin, GPIO.LOW)
	pass
	GPIO.output(csnpin, GPIO.HIGH)
	pass
	GPIO.output(clkpin, GPIO.HIGH)
	#
	return (chainData >> 6)

begincom()

starttime = time.time()

for x in range(1000):
	var = readenc()
	pass

endtime = time.time()

print(endtime - starttime)