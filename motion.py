#!/usr/bin/env python

from __future__ import division
import time
import serial
import Adafruit_PCA9685
import time
import RPi.GPIO as GPIO
import datetime
import subprocess
import ConfigParser
import sys

DEBUG = 0

LEFT  = 0
RIGHT = 1

STP = 0
RWD = 1
FWD = 2

IDLE    = 0
MOVE    = 1
EXIT    = 2
WAIT    = 3
CALIB   = 4
MANUAL  = 5
CONTROL = 6
INIT    = 7

OFFSETS = [0, 0]    # overwritten by ini file settings
MIDDLE = 400        # overwritten by ini file settings


def run(l, r):
    global pwmL, pwmR
    pwmL = MIDDLE + OFFSETS[LEFT] + l
    pwmR = MIDDLE - OFFSETS[RIGHT] - r
    pwm.set_pwm(LEFT, 0, pwmL)
    pwm.set_pwm(RIGHT, 0, pwmR)
    #print("PWMs: "+str(pwmL)+", "+str(pwmR))

def freerun():
    pwm.set_pwm(LEFT, 0, 0)
    pwm.set_pwm(RIGHT, 0, 0)

def resetEncoder():
    global encoderR, encoderL

    print("Reset encoder")
    encoderL = 0
    encoderR = 0

def readIni():
    global OFFSETS
    Config = ConfigParser.ConfigParser()
    Config.read("config.ini")
    OFFSETS[LEFT] = Config.getint('motor', 'offset_left')
    OFFSETS[RIGHT] = Config.getint('motor', 'offset_right')
    MIDDLE = Config.getint('motor', 'middle')
    #print("Offset Left: "+str(OFFSETS[LEFT]))

def trace():
    print("pwmL: "+str(pwmL)+", pwmR: "+str(pwmR)+", SpeedL: "+str(speedL)+", SpeedR: "+str(speedR)+", encoderL: "+str(encoderL)+", encoderR: "+str(encoderR))
    if (DEBUG == 1):
        f.write(str(time.time())+";"+str(pwmL)+";"+str(pwmR)+";"+str(speedL)+";"+str(speedR)+";"+str(encoderL)+";"+str(encoderR)+"\r\n")

def startup():
    global state
    if(sys.argv[1]=="m"):
        state = MANUAL
    elif(sys.argv[1]=="cal"):
        state = CALIB
    elif(sys.argv[1] == "ctrl"):
        state = CONTROL
    elif(sys.argv[1]=="i"):
        state = INIT
    else:
        state = IDLE

# PI control with additional position control
def control(speed_left, speed_right, pulses_left, pulses_right, timeout=30, positionControl=1):
    global e_speedL_I, e_speedR_I, e_pos_I
    pl = 0
    pr = 0
    sl = speed_left
    sr = speed_right
    Kp = 1
    Ki = 0.5
    Ko = 2
    t = time.time()
    while((time.time() - t) < timeout):
        e_speedL = sl-speedL
        e_speedR = sr-speedR
        e_speedL_I += e_speedL
        e_speedR_I += e_speedR
        e_pos_I += encoderL - encoderR
        if(speed_left != 0):
            pl = Kp*e_speedL + Ki*e_speedL_I
        else:
            positionControl = 0
        if(speed_right != 0):
            pr = Kp*e_speedR + Ki*e_speedR_I + Ko*e_pos_I*positionControl
        print("Pl: "+str(pl)+", Pr: "+str(pr))
        run(int(pl), int(pr))
        if( (encoderL == pulses_left) or (encoderR == pulses_right) ):
            break
        time.sleep(0.05)
        speedMonitor()
        trace()
    freerun()

# Main script
###########################################
#
#### INIT #################################
# Initialise the PCA9685 using the default address (0x40).
pwm = Adafruit_PCA9685.PCA9685()
pwm.set_pwm_freq(60)
ser = serial.Serial('/dev/ttyS0', 115200, timeout=0)
state = IDLE
lasttime = time.time()
encoderL = 0
encoderR = 0
directionL = STP
directionR = STP
lastPulseLa = 0
lastPulseRa = 0
lastPulseLb = 0
lastPulseRb = 0
speedLa = 0
speedRa = 0
speedLb = 0
speedRb = 0
speedL = 0
speedR = 0
pwmL = 0
pwmR = 0
calibL = 0
calibR = 0
e_speedL_I = 0
e_speedR_I = 0
e_pos_I = 0

#initEncoder()
readIni()
startup()

if(DEBUG == 1):
    f= open("trace.txt","w+")
    f.write("timestamp;pwmL;pwmR;speedLa;speedRa;encoderL;encoderR\r\n")
#
#### MAIN #################################
#
try:
    while True:
        # Handle states
        if(state==INIT):
            control(0, 2, 1, 5, 0)
            time.sleep(1)
            trace()
            resetEncoder()
            trace()
            control(2, 0, 1, 5, 0)
            time.sleep(1)
            trace()
            resetEncoder()
            trace()
            state = IDLE
        if(state==IDLE):
            #print("Idle state")
            time.sleep(0.1)
            state = EXIT
        elif(state==MANUAL):
            pl = input("power left ?")
            pr = input("power right ?")
            #print "pwm L: ", pl, ", pwm R: ", pr
            run(pl, pr)
            t = time.time()
            while((time.time() - t)<2):
                time.sleep(0.1)
                line = ser.readline()   #read a '\n' terminated line
                data = line.split(';')
                if(data[0] == "MOV"):
                    encoderL = int(data[1])
                    encoderR = int(data[2])
                    speedL = int(data[3])
                    speedR = int(data[4])
                if(data[0] == "DBG"):
                    print line
                trace()
            freerun()
            #state = EXIT
        elif(state==CONTROL):
            print("Control state")
            #control(5, 5, 30, 60)
            s  = input("speed ?")
            pl = input("steps left ?")
            pr = input("steps right ?")
            control(s, s, pl, pr, 10, 1)
            state = EXIT
        elif(state==CALIB):
            run(calibL, calibR)
            print("Calib L: "+str(calibL)+", Calib R: "+str(calibR)+", SpeedL: "+str(speedL)+", SpeedR: "+str(speedR)+", encoderL: "+str(encoderL)+", encoderR: "+str(encoderR))
            if(speedL>=5 and speedR>=5):
                print("calibration completed.")
                state = EXIT
            else:
                if(speedL>0 or speedR>0):
                    if(speedR>speedL):
                        calibL += 1
                    else:
                        calibR += 1
                else:
                    calibL += 1
                    calibR += 1
            time.sleep(0.4)
            trace()
        elif(state==EXIT):
            #print "State EXIT"
            #state = IDLE
            freerun()
            ser.close()
            break
# Cyclic statements
#        time.sleep(0.1)

except KeyboardInterrupt:
    print "Finished"
    #GPIO.cleanup()
    freerun()
    if (DEBUG == 1):
        f.close()
