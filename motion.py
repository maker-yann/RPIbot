#!/usr/bin/env python

from __future__ import division
import time
import Adafruit_PCA9685
import time
import RPi.GPIO as GPIO
import datetime
import subprocess
import ConfigParser

# Left encoder
ENCODER_0A = 35
ENCODER_0B = 37
# Right encoder
ENCODER_1A = 31
ENCODER_1B = 33

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

def encoder0(channel):
    global encoderL, directionL, lastPulseL, speedL
    #print('Edge L detected on channel %s'%channel)
    a = GPIO.input(ENCODER_0A)
    b = GPIO.input(ENCODER_0B)

    if( ( (channel == ENCODER_0A) and (a == GPIO.HIGH) and (b == GPIO.LOW) )
      or ( (channel == ENCODER_0A) and (a == GPIO.LOW) and (b == GPIO.HIGH) )
      or ( (channel == ENCODER_0B) and (a == GPIO.HIGH) and (b == GPIO.HIGH) )
      or ( (channel == ENCODER_0B) and (a == GPIO.LOW) and (b == GPIO.LOW) ) ):
        if(directionL == RWD): # same direction as previous pulse
            speedL = -1/(time.time() - lastPulseL) # assuming only one pulse more
            #print("SpeedL: " + str(speedL))
        encoderL -= 1
        directionL = RWD
    else:
        if(directionL == FWD): # same direction as previous pulse
            speedL = 1/(time.time() - lastPulseL) # assuming only one pulse more
            #print("SpeedL: " + str(speedL))
        encoderL += 1
        directionL = FWD
    lastPulseL = time.time()
    #print("t: "+str(time.time())+", a: "+str(a)+", b: "+str(b)+", EncoderL: "+str(encoderL))
    trace()

def encoder1(channel):
    global encoderR, directionR, lastPulseR, speedR
    #print('Edge R detected on channel %s'%channel)
    a = GPIO.input(ENCODER_1A)
    b = GPIO.input(ENCODER_1B)

    if( ( (channel == ENCODER_1A) and (a == GPIO.HIGH) and (b == GPIO.LOW) )
      or ( (channel == ENCODER_1A) and (a == GPIO.LOW) and (b == GPIO.HIGH) )
      or ( (channel == ENCODER_1B) and (a == GPIO.HIGH) and (b == GPIO.HIGH) )
      or ( (channel == ENCODER_1B) and (a == GPIO.LOW) and (b == GPIO.LOW) ) ):
        if(directionR == FWD): # same direction as previous pulse
            speedR = 1/(time.time() - lastPulseR) # assuming only one pulse more
            #print("SpeedR: " + str(speedR))
        encoderR += 1
        directionR = FWD
    else:
        if(directionR == RWD): # same direction as previous pulse
            speedR = -1/(time.time() - lastPulseR) # assuming only one pulse more
            #print("SpeedR: " + str(speedR))
        encoderR -= 1
        direction = RWD
    lastPulseR = time.time()
    print("t: "+str(time.time())+", a: "+str(a)+", b: "+str(b)+", EncoderR: "+str(encoderR))
    trace()

def initEncoder():
    GPIO.setmode(GPIO.BOARD) # Alternative: GPIO.BCM

    GPIO.setup(ENCODER_0A, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    GPIO.setup(ENCODER_0B, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    GPIO.setup(ENCODER_1A, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    GPIO.setup(ENCODER_1B, GPIO.IN, pull_up_down=GPIO.PUD_UP)

    GPIO.add_event_detect(ENCODER_0A, GPIO.BOTH, encoder0)
    GPIO.add_event_detect(ENCODER_0B, GPIO.BOTH, encoder0)
    GPIO.add_event_detect(ENCODER_1A, GPIO.BOTH, encoder1)
    GPIO.add_event_detect(ENCODER_1B, GPIO.BOTH, encoder1)

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
    #print("pwmL: "+str(calibL)+", pwmR: "+str(calibR)+", SpeedL: "+str(speedL)+", SpeedR: "+str(speedR)+", encoderL: "+str(encoderL)+", encoderR: "+str(encoderR))
    if (DEBUG == 1):
        f.write(str(time.time())+";"+str(pwmL)+";"+str(pwmR)+";"+str(speedL)+";"+str(speedR)+";"+str(encoderL)+";"+str(encoderR)+"\r\n")


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
speedL = 0
speedR = 0
pwmL = 0
pwmR = 0
calibL = 0
calibR = 0

initEncoder()
readIni()
if(DEBUG == 1):
    f= open("trace.txt","w+")
    f.write("timestamp;pwmL;pwmR;speedL;speedR;encoderL;encoderR\r\n")
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
            #print("Calib L: "+str(calibL)+", Calib R: "+str(calibR)+", SpeedL: "+str(speedL)+", SpeedR: "+str(speedR)+", encoderL: "+str(encoderL)+", encoderR: "+str(encoderR))
            if(speedL>=15 and speedR>=15):
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
            #print "State EXIT started"
            #state = IDLE
            freerun()
            pass
        elif(state==WAIT):
            pass
        time.sleep(0.1)
        if(time.time() - lastPulseL)>1:
            #print("speedL: 0")
            speedL = 0
        if(time.time() - lastPulseR)>1:
            #print("speedR: 0")
            speedR = 0

except KeyboardInterrupt:
    print "Finished"
    #GPIO.cleanup()
    freerun()
    if (DEBUG == 1):
        f.close()
