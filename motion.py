#!/usr/bin/env python

from __future__ import division
import time
import Adafruit_PCA9685
import time
import RPi.GPIO as GPIO
import datetime
import subprocess
import ConfigParser

DEBUG = 0

LEFT  = 0
RIGHT = 1

STP = 0
RWD = 1
FWD = 2

IDLE = 0
MOVE = 1
EXIT = 2
WAIT = 3
CALIB= 4

OFFSETS = [0, 0]    # overwritten by ini file settings
MIDDLE = 400        # overwritten by ini file settings


# max +/- 205
def run(l, r):
    global pwmL, pwmR
    pwmL = MIDDLE + OFFSETS[LEFT] + l
    pwmR = MIDDLE - OFFSETS[RIGHT] - r
    pwm.set_pwm(LEFT, 0, pwmL)
    pwm.set_pwm(RIGHT, 0, pwmR)
    #print("PWMs: "+str(pwm_left)+", "+str(pwm_right))

def freerun():
    pwm.set_pwm(LEFT, 0, 0)
    pwm.set_pwm(RIGHT, 0, 0)

def readIni():
    global OFFSETS
    Config = ConfigParser.ConfigParser()
    Config.read("config.ini")
    OFFSETS[LEFT] = Config.getint('motor', 'offset_left')
    OFFSETS[RIGHT] = Config.getint('motor', 'offset_right')
    MIDDLE = Config.getint('motor', 'middle')
    #print("Offset Left: "+str(OFFSETS[LEFT]))

def trace():
    print("pwmL: "+str(calibL)+", pwmR: "+str(calibR)+", SpeedLa: "+str(speedLa)+", SpeedRa: "+str(speedRa)+", encoderL: "+str(encoderL)+", encoderR: "+str(encoderR))
    if (DEBUG == 1):
        f.write(str(time.time())+";"+str(pwmL)+";"+str(pwmR)+";"+str(speedLa)+";"+str(speedRa)+";"+str(encoderL)+";"+str(encoderR)+"\r\n")


# Main script
###########################################
#
#### INIT #################################
# Initialise the PCA9685 using the default address (0x40).
pwm = Adafruit_PCA9685.PCA9685()
pwm.set_pwm_freq(60)
state = IDLE
lasttime = time.time()
encoderL = 0
encoderR = 0
directionL = STP
directionR = STP
lastPulseL = 0
lastPulseR = 0
speedLa = 0
speedRa = 0
speedLb = 0
speedRb = 0
pwmL = 0
pwmR = 0
calibL = 0
calibR = 0

readIni()

if(DEBUG == 1):
    f= open("trace.txt","w+")
    f.write("timestamp;pwmL;pwmR;speedLa;speedRa;encoderL;encoderR\r\n")
#
#### MAIN #################################
#
try:
    while True:
        # Handle states
        if(state==IDLE):
            pl = input("power left ?")
            pr = input("power right ?")
            print "pwm L: ", pl, ", pwm R: ", pr
            run(pl, pr)
            t = time.time()
            while((time.time() - t)<3):
                time.sleep(0.1)
                trace()
            freerun()
            #state = EXIT
        elif(state==CALIB):
            run(calibL, calibR)
            #print("Calib L: "+str(calibL)+", Calib R: "+str(calibR)+", SpeedLa: "+str(speedLa)+", SpeedRa: "+str(speedRa)+", encoderL: "+str(encoderL)+", encoderR: "+str(encoderR))
            if(speedLa>=15 and speedRa>=15):
                print("calibration completed.")
                state = EXIT
            else:
                if(speedLa>0 or speedRa>0):
                    if(speedRa>speedLa):
                        calibL += 1
                    else:
                        calibR += 1
                else:
                    calibL += 1
                    calibR += 1
            time.sleep(0.4)
            trace()
        elif(state==EXIT):
            #print "State EXIT started"
            #state = IDLE
            freerun()
            pass
        elif(state==WAIT):
            pass
        time.sleep(0.1)
        if(time.time() - lastPulseL)>1:
            #print("speedL: 0")
            speedLa = 0
        if(time.time() - lastPulseR)>1:
            #print("speedR: 0")
            speedRa = 0

except KeyboardInterrupt:
    print "Finished"
    #GPIO.cleanup()
    freerun()
    if (DEBUG == 1):
        f.close()
