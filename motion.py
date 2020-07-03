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

LVL_NONE            = 70
LVL_CRITICAL        = 60
LVL_ERROR           = 50
LVL_WARNING         = 40
LVL_INFORMATION     = 30
LVL_DEBUGGING       = 20
LVL_ALL             = 10

DEBUG = LVL_INFORMATION

TRACE       = 0

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

# low level abstraction
# l, r: PWMs (direction, middle point and offset are corrected here)
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
    #global encoderR, encoderL
    global leftEncoderZero, rightEncoderZero

    logEvent(LVL_INFORMATION, "Reset encoder")
    #encoderL = 0
    #encoderR = 0
    leftEncoderZero += encoderL
    rightEncoderZero += encoderR

def readIni():
    global OFFSETS

    logEvent(LVL_INFORMATION, "Read ini file")
    Config = ConfigParser.ConfigParser()
    Config.read("config.ini")
    OFFSETS[LEFT] = Config.getint('motor', 'offset_left')
    OFFSETS[RIGHT] = Config.getint('motor', 'offset_right')
    MIDDLE = Config.getint('motor', 'middle')
    #print("Offset Left: "+str(OFFSETS[LEFT]))

def initLog():
    global logfile
    if(DEBUG < LVL_NONE):
        logfile = open("robot.log","w+")
        logfile.write("Starting session\r\n")

def initTrace():
    global tracefile
    if(TRACE == 1):
        tracefile = open("trace.txt","w+")
        tracefile.write("timestamp;pwmL;pwmR;speedL;speedR;encoderL;encoderR\r\n")

def writeTrace():
    if (DEBUG <= LVL_DEBUGGING):
        print("pwmL: "+str(pwmL)+", pwmR: "+str(pwmR)+", SpeedL: "+str(speedL)+", SpeedR: "+str(speedR)+", encoderL: "+str(encoderL)+", encoderR: "+str(encoderR))
    if (TRACE == 1):
        tracefile.write(str(time.time())+";"+str(pwmL)+";"+str(pwmR)+";"+str(speedL)+";"+str(speedR)+";"+str(encoderL)+";"+str(encoderR)+"\r\n")

def logEvent(level, event):
    if (level >= LVL_INFORMATION):
        print event
        logfile.write(str(time.time()) + ";" + event + "\r\n")

def readCommand():
    global state

    c = raw_input("""Command (man, ctrl, cal, exit, )?: """)
    logEvent(LVL_INFORMATION, "Command by user: " + str(c))
    if(c == "man"):
        state = MANUAL
    elif(c == "ctrl"):
        state = CONTROL
    elif(c == "cal"):
        state = CALIB
    elif(c == "exit"):
        state = EXIT
    else:
        state = EXIT

def readEncoder():
    global encoderL, encoderR, speedL, speedR
    success = False
    line = ser.readline()   #read a '\n' terminated line (timeout set in open statement)
    data = line.split(';')
    if(data[0] == "MOV"):
        encoderL = int(data[1]) - leftEncoderZero
        encoderR = int(data[2]) - rightEncoderZero
        speedL = int(data[3])
        speedR = int(data[4])
        writeTrace()
        success = True
    if(data[0] == "DBG"):
        logEvent(LVL_DEBUGGING, line)
    return success

# Main script
###########################################
#
#### INIT #################################
# Initialise the PCA9685 using the default address (0x40).
pwm = Adafruit_PCA9685.PCA9685()
pwm.set_pwm_freq(60)
ser = serial.Serial('/dev/ttyS0', 115200, timeout=0.1)
state = INIT
lastState = IDLE
lasttime = time.time()
encoderL = 0
encoderR = 0
leftEncoderZero = 0
rightEncoderZero = 0
directionL = STP
directionR = STP
speedL = 0
speedR = 0
pwmL = 0
pwmR = 0
calibL = 0
calibR = 0
e_speedL_I = 0
e_speedR_I = 0
e_pos_I = 0
pl = 0 # power in PWM unit
pr = 0 # power in PWM unit

initLog()
logEvent(LVL_INFORMATION, "Starting session")
readIni()

#
#### MAIN #################################
#
try:
    while True:
        # Handle states
        if(state==INIT):
            readCommand()
        elif(state==IDLE):
            #print("Idle state")
            time.sleep(0.1)
            state = EXIT
        elif(state==MANUAL):
            logEvent(LVL_INFORMATION, "Starting manual mode")
            pl = input("power left ?")
            pr = input("power right ?")
            duration = input("duration ?")
            resetEncoder()
            #print "pwm L: ", pl, ", pwm R: ", pr
            run(pl, pr)
            ser.flushInput()
            t = time.time()
            while((time.time() - t) < int(duration)):
                readEncoder()
            freerun()
            state = IDLE
        elif(state==CONTROL):
            logEvent(LVL_INFORMATION, "Starting Control mode")
            #control(5, 5, 30, 60)
            s  = input("speed ?")
            #pl = input("steps left ?")
            #pr = input("steps right ?")
            #control(s, s, pl, pr, 3, 0)
            ser.flushInput()
            t = time.time()
            while((time.time() - t)<10):
                if(readEncoder() == True):
                    if(speedL < int(s)):
                        pl = pl + 1
                    elif(speedL > int(s)+15):
                        pl = pl - 1
                    if(speedR < int(s)):
                        pr = pr + 1
                    elif(speedR > int(s)+15):
                        pr = pr - 1
                    run(pl, pr)
            state = EXIT
        elif(state==CALIB):
            logEvent(LVL_INFORMATION, "Starting calibration mode")
            ser.flushInput()
            for i in range(-15, 15):
                t = time.time()
                meanSpeedLeft  = 0
                meanSpeedRight = 0
                nb_values = 0
                run(i, i)
                while((time.time() - t) < 3):
                    if(readEncoder() == True):
                        meanSpeedLeft  += speedL
                        meanSpeedRight += speedR
                        nb_values += 1
                        logEvent(LVL_DEBUGGING, "CAL: Left [PWM="+str(i)+"; speed="+str(speedL)+"], Right [PWM="+str(i)+"; speed="+str(speedR)+"]")
                meanSpeedLeft = meanSpeedLeft/nb_values
                meanSpeedRight = meanSpeedRight/nb_values
                logEvent(LVL_INFORMATION, "CAL: Left [PWM="+str(i)+"; speed="+str(meanSpeedLeft)+"], Right [PWM="+str(i)+"; speed="+str(meanSpeedRight)+"]")
            state = EXIT
        elif(state==EXIT):
            #print "State EXIT"
            #state = IDLE
            freerun()
            ser.close()
            break
        if(state != lastState):
            logEvent(LVL_INFORMATION, "State change to: " + str(state))
        lastState = state

# Cyclic statements
#        time.sleep(0.1)

except KeyboardInterrupt:
    print "Finished"
    #GPIO.cleanup()
    freerun()
    ser.close()
    if(DEBUG < LVL_NONE):
        logfile.close()
    if(TRACE == 1):
        tracefile.close()
