#!/usr/bin/env python
# -*- coding: utf8 -*-

import Adafruit_BBIO.PWM as PWM
import time
usleep = lambda x: time.sleep( x / 1000000.0)

PWM.start('P8_13', 92.5, 50, 1)

def duty(value):
	return ((2.5 * value) / 100 + 92.5)

starttime = time.time()

for x in range(1000):
	PWM.set_duty_cycle('P8_13', duty((2 * x / 10) - 100))
	usleep(19.9 * 1000)

endtime = time.time()

print(endtime - starttime)