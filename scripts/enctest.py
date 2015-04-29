import Adafruit_BBIO.GPIO as GPIO
import time
usleep = lambda x: time.sleep(x/1000000.0)
cs1 = "P9_31"
cs2 = "P9_29"
do  = "P9_25"
clk = "P9_23"
GPIO.cleanup()
GPIO.setup(cs1, GPIO.OUT)
GPIO.setup(cs2, GPIO.OUT)
GPIO.setup(do,  GPIO.IN)
GPIO.setup(clk, GPIO.OUT)

def closeComm(pincsn):
    GPIO.output(pincsn,   GPIO.HIGH)
    GPIO.output(clk,      GPIO.HIGH)
    #

def readSingle(pincsn):
    chainData = 0
    GPIO.output(pincsn,   GPIO.LOW)
    usleep(0)
    GPIO.output(clk,      GPIO.LOW)
    usleep(0)
    for k in range(16):
        GPIO.output(clk, GPIO.HIGH)
        usleep(0)
        chainData = (chainData << 1) | GPIO.input(do)
        GPIO.output(clk, GPIO.LOW)
        usleep(0)
        #
    #
    closeComm(pincsn)
    return (chainData >> 6)
    #

closeComm(cs1)
closeComm(cs2)

readSingle(cs1)