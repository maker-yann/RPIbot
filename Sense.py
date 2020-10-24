#!/usr/bin/env python
from __future__ import division

import time
import logging
import serial
from threading import Thread
#import Adafruit_PCA9685
from mpu9250_i2c import *
import RPi.GPIO as GPIO
import datetime
import subprocess
import ConfigParser
import sys
import json
from functools import partial


logger = logging.getLogger(__name__)
logger.setLevel(logging.DEBUG)
# create console handler with a higher log level
##ch = logging.StreamHandler()
##ch.setLevel(logging.DEBUG)
# create formatter and add it to the handlers
##formatter = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s', datefmt='%Y/%m/%d %H:%M:%S')
##ch.setFormatter(formatter)
# add the handlers to the logger
##logger.addHandler(ch)

class Sense(Thread):

    # Definition of class variables. Whould be nice to shift them in __init__ as instance variables.
    TRACE = 0
    SERIAL = 0
    IMU = 1
    
    STP  =  0
    RWD  = -1
    FWD  =  1

    GPIO_ENCODER_L = 16
    GPIO_ENCODER_R = 20

    #### INIT #################################
    def __init__(self):
        Thread.__init__(self) 
        self._running = True
        if(self.SERIAL == 1):
            self.ser = serial.Serial('/dev/ttyS0', 115200, timeout=0.1)
        self.line = ''
        self.encoderL = 0
        self.encoderR = 0
        self.directionL = self.STP
        self.directionR = self.STP
        self.raw_encoder_L = 0
        self.raw_encoder_R = 0
        #self.leftEncoderZero = 0
        #self.rightEncoderZero = 0
        self.speedL = 0
        self.speedR = 0
        self.distance = 0
        self.lastFrameTimestamp = time.time()
        self.cycleTime = 0
        self.RT_data = {} # Real time data
        self.speedTableL = [0, 0, 0]
        self.speedTableR = [0, 0, 0]

        self.lastPulseL = time.time()
        self.lastPulseR = time.time()
        
        self.initEncoder()

    def initEncoder(self):
        GPIO.setmode(GPIO.BCM)

        GPIO.setup(self.GPIO_ENCODER_L, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(self.GPIO_ENCODER_R, GPIO.IN, pull_up_down=GPIO.PUD_UP)

        GPIO.add_event_detect(self.GPIO_ENCODER_L, GPIO.RISING, self.callbackEncoderL)
        GPIO.add_event_detect(self.GPIO_ENCODER_R, GPIO.RISING, self.callbackEncoderR)

    def callbackEncoderL(self, channel):
        self.encoderL += 1 * self.directionL #consider also encoder direction
        self.speedL = int((1.075*2)/(time.time() - self.lastPulseL))
        self.lastPulseL = time.time()

    def callbackEncoderR(self, channel):
        self.encoderR += 1 * self.directionR
        self.speedR = int((1.075*2)/(time.time() - self.lastPulseR)) # cm/s (only rising edge)
        self.lastPulseR = time.time()

    def readSerial(self):
        res = False

        try:
            if(self.SERIAL == 1):
                self.line = self.ser.readline()   #read a '\n' terminated line (timeout set in open statement)
                if "MOV" in self.line:
                    self.cycleTime = int(round((time.time() - self.lastFrameTimestamp)*1000))
                    self.lastFrameTimestamp = time.time()
                res = True
            #print(self.line)
        except:
            logger.error("Serial readline error: "+self.line)
        return res

    def readGPIOData(self):
        self.RT_data["encoderL"] = self.encoderL
        self.RT_data["encoderR"] = self.encoderR
        self.RT_data["odoDistance"] = (self.encoderL + self.encoderR)/2
        self.RT_data["yaw"] = self.encoderL - self.encoderR
        if (time.time() - self.lastPulseL) > 0.5:
            self.RT_data["speedL"] = 0.0
        else:
            self.RT_data["speedL"] = self.speedL
        if (time.time() - self.lastPulseR) > 0.5:
            self.RT_data["speedR"] = 0.0
        else:
            self.RT_data["speedR"] = self.speedR
        self.cycleTime = int(round((time.time() - self.lastFrameTimestamp)*1000))
        self.RT_data["cycleTimeSense"] = self.cycleTime
        self.lastFrameTimestamp = time.time()

    def outputData(self):
        print (str(self.cycleTime) + " - RT_Data: ", str(self.RT_data))

    def decodeSerialFrame(self):
        try:
            data = self.line.split(';')
            if(data[0] == "MOV"):
                self.encoderL = int(data[1])
                self.encoderR = int(data[2])
                self.speedL = int(data[3])
                self.speedR = int(data[4])
                if("55" not in data[5]):
                    logger.error("Wrong control value in MOV")
                self.RT_data["encoderL"] = self.encoderL
                self.RT_data["encoderR"] = self.encoderR
                self.RT_data["speedL"] = self.speedL
                self.RT_data["speedR"] = self.speedR
            elif(data[0] == "DBG"):
                self.raw_encoder_L = int(data[1])
                self.raw_encoder_R = int(data[2])
                if("55" not in data[3]):
                    logger.error("Wrong control value in DBG")
                self.RT_data["raw_encoderL"] = self.raw_encoder_L
                self.RT_data["raw_encoderR"] = self.raw_encoder_R
            elif(data[0] == "DST"):
                self.distance = int(data[1])
                if("55" not in data[2]):
                    logger.error("Wrong control value in DST")
                self.RT_data["distance"] = self.distance
        except:
            logger.error("Frame not complete: "+self.line) # serial data was not complete

    def readIMU(self):
        if(self.IMU == 1):
            ax, wz, hdg, m_x, m_y = mpu9250_read()
            self.RT_data["ax"] = ax
            self.RT_data["wz"] = wz
            self.RT_data["hdg"] = hdg
            self.RT_data["m_x"] = m_x
            self.RT_data["m_y"] = m_y
            #print('accel [g]: x = {0:.2f}, wz = {1:.2f} , hdg = {2:.2f} '.format(ax,wz,hdg))

    def close(self):
        logger.info("Closing serial")
        if(self.SERIAL == 1):
            self.ser.close()

    def terminate(self):
        self._running = False

    def run(self):
        time.sleep(0.1)
        self.readSerial()
        while self._running:
            if self.readSerial():
                self.decodeSerialFrame()
            self.readGPIOData()
            self.readIMU() ## Too long. Shift in a thread !!
            #time.sleep(0.1)
        self.close()

# Run this if standalone (test purpose)
if __name__ == '__main__':
    # create console handler with a higher log level
    ch = logging.StreamHandler()
    ch.setLevel(logging.DEBUG)
    # create formatter and add it to the handlers
    formatter = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s', datefmt='%Y/%m/%d %H:%M:%S')
    ch.setFormatter(formatter)
    # add the handlers to the logger
    logger.addHandler(ch)

    try:
        logger.info("Started main")
        s = Sense()
        s.start()
        while(1):
            s.outputData()
            time.sleep(0.1)
    except KeyboardInterrupt:
        # Signal termination
        logger.info("Keyboard interrupt. Terminate thread")
        s.terminate()
        logger.debug("Thread terminated")

        # Wait for actual termination (if needed)
        s.join()
        logger.debug("Thread finished")

