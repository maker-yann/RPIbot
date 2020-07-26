from __future__ import division
import logging
import time
import serial
import Adafruit_PCA9685
import time
import RPi.GPIO as GPIO
import datetime
import subprocess
import ConfigParser
import sys
import json
import threading

class motioncontrol:
    # Definition of class variables. Whould be nice to shift them in __init__ as instance variables.
    TRACE = 0

    LEFT  = 0
    RIGHT = 1
    PAN   = 2
    TILT  = 3

    TURN =  0
    STP  =  0
    RWD  = -1
    FWD  =  1

    IDLE    = 0
    MOVE    = 1
    EXIT    = 2
    WAIT    = 3
    CALIB   = 4
    MANUAL  = 5
    CONTROL = 6
    INIT    = 7
    HEAD    = 8
    TEST    = 9
    DRIVE   = 10

    OFFSETS = [0, 0, 0, 0]    # overwritten by ini file settings
    MIDDLE = 400        # overwritten by ini file settings
    STOP_DISTANCE = 50
    SPEED_HYSTERESIS = 15
    PWM_INCREMENT = 2
    YAW_INCREMENT = 2
    OFFSET_MOVE = 10

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
    raw_encoder_L = 0
    raw_encoder_R = 0
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
    distance = 0
    RT_data = {} # Real time data


    def __init__(self):
        self.logger = logging.getLogger(__name__)
        
        self.logger.setLevel(logging.DEBUG)
        # create file handler which logs even debug messages
        fh = logging.FileHandler('rpibot.log')
        fh.setLevel(logging.INFO)
        # create console handler with a higher log level
        #ch = logging.StreamHandler()
        #ch.setLevel(logging.DEBUG)
        # create formatter and add it to the handlers
        formatter = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s', datefmt='%Y/%m/%d %H:%M:%S')
        fh.setFormatter(formatter)
        #ch.setFormatter(formatter)
        # add the handlers to the logger
        self.logger.addHandler(fh)
        #self.logger.addHandler(ch)
        self.readIni()
        self.freerun()

    def close(self):
        self.logger.warning("Closing motioncontrol")
        self.freerun()
        self.ser.close()
        if(self.TRACE == 1):
            tracefile.close()

    def readIni(self):
        self.logger.info("Read ini file")
        Config = ConfigParser.ConfigParser()
        Config.read("config.ini")
        self.OFFSETS[self.LEFT] = Config.getint('motor', 'offset_left')
        self.OFFSETS[self.RIGHT] = Config.getint('motor', 'offset_right')
        self.OFFSETS[self.PAN] = Config.getint('motor', 'offset_pan')
        self.OFFSETS[self.TILT] = Config.getint('motor', 'offset_tilt')
        self.MIDDLE = Config.getint('motor', 'middle')

    def initTrace():
        if(self.TRACE == 1):
            tracefile = open("trace.txt","w+")
            tracefile.write("timestamp;pwmL;pwmR;speedL;speedR;encoderL;encoderR\r\n")

    def writeTrace(self):
        self.logger.debug("pwmL: "+str(self.pwmL)+", pwmR: "+str(self.pwmR)+", SpeedL: "+str(self.speedL)+", SpeedR: "+str(self.speedR)+", encoderL: "+str(self.encoderL)+", encoderR: "+str(self.encoderR))
        if (self.TRACE == 1):
            tracefile.write(str(time.time())+";"+str(self.pwmL)+";"+str(self.pwmR)+";"+str(self.speedL)+";"+str(self.speedR)+";"+str(self.encoderL)+";"+str(self.encoderR)+"\r\n")

    def readEncoder(self):
        #self.ser.flushInput()
        success = False
        line = ''
        received_mov = False
        received_dbg = False
        received_dst = False
        t = time.time()
        while((time.time() - t) < 1):
            try:
                line = self.ser.readline()   #read a '\n' terminated line (timeout set in open statement)
                #print(line.strip()+'\r')
            except:
                self.logger.error("Serial readline error: "+line)
            else:
                try:    
                    data = line.split(';')
                    if(data[0] == "MOV"):
                        self.encoderL = int(data[1]) - self.leftEncoderZero
                        self.encoderR = int(data[2]) - self.rightEncoderZero
                        self.speedL = int(data[3])
                        self.speedR = int(data[4])
                        if("55" not in data[5]):
                            self.logger.error("Wrong control value in MOV")
                        self.RT_data["encoderL"] = self.encoderL
                        self.RT_data["encoderR"] = self.encoderR
                        self.RT_data["speedL"] = self.speedL
                        self.RT_data["speedR"] = self.speedR
                        received_mov = True
                    elif(data[0] == "DBG"):
                        #self.logger.debug("Rx: "+line)
                        self.raw_encoder_L = int(data[1])
                        self.raw_encoder_R = int(data[2])
                        if("55" not in data[3]):
                            self.logger.error("Wrong control value in DBG")
                        self.RT_data["raw_encoderL"] = self.raw_encoder_L
                        self.RT_data["raw_encoderR"] = self.raw_encoder_R
                        received_dbg = True
                    elif(data[0] == "DST"):
                        self.distance = int(data[1])
                        if("55" not in data[2]):
                            self.logger.error("Wrong control value in DST")
                        self.RT_data["distance"] = self.distance
                        received_dst = True
                        #self.logger.debug("DST: "+str(self.distance))
                except:
                    self.logger.error("Frame not complete: "+line) # serial data was not complete
                if( (received_mov == True) and (received_dbg == True) and (received_dst == True) ):
                    success = True
                    #self.logger.debug("E_l: %d; E_r: %d; S_l: %d; S_r: %d", self.encoderL, self.encoderR, self.speedL, self.speedR)
                    break
        if(success == False):
            self.logger.error("Serial frame timeout")
        return success
        
    def startMove(self, leftSpeedTarget, rightSpeedTarget, leftEncoderTarget, rightEncoderTarget, timeout):
        self.leftSpeedTarget = leftSpeedTarget
        self.rightSpeedTarget = rightSpeedTarget
        self.leftEncoderTarget = leftEncoderTarget
        self.rightEncoderTarget = rightEncoderTarget
        self.timeout = timeout
        
        self.logger.info("Calling MOVE with left speed = "+str(leftSpeedTarget)+", right speed = "+str(rightSpeedTarget)+", left encoder = "+str(leftEncoderTarget)+", right encoder = "+str(rightEncoderTarget))
#        self.ser.flushInput()
        self.t = time.time()
        if(leftSpeedTarget > 0):
            self.directionL = self.FWD
        elif(leftSpeedTarget < 0):
            self.directionL = self.RWD
        else:
            self.directionL = self.STP
        if(rightSpeedTarget > 0):
            self.directionR = self.FWD
        elif(rightSpeedTarget < 0):
            self.directionR = self.RWD
        else:
            self.directionR = self.STP
        self.pl = self.OFFSET_MOVE*self.directionL
        self.pr = self.OFFSET_MOVE*self.directionR
        self.yawCorrection = 0
                    
    def cyclicMove(self):
        if((time.time() - self.t) < self.timeout):
            if(self.directionL*self.speedL < int(self.leftSpeedTarget)):
                self.pl = self.pl + self.PWM_INCREMENT
            elif(self.directionL*self.speedL > int(self.leftSpeedTarget)+self.SPEED_HYSTERESIS):
                self.pl = self.pl - self.PWM_INCREMENT
            if(self.directionR*self.speedR < int(self.rightSpeedTarget)):
                self.pr = self.pr + self.PWM_INCREMENT
            elif(self.directionR*self.speedR > int(self.rightSpeedTarget)+self.SPEED_HYSTERESIS):
                self.pr = self.pr - self.PWM_INCREMENT
            #if( (leftEncoderTarget == rightEncoderTarget) and (directionL*directionR>0)):
            if(self.leftEncoderTarget == self.rightEncoderTarget):
                if(self.encoderL > self.encoderR):
                    #self.yawCorrection += self.YAW_INCREMENT
                    self.yawCorrection = self.YAW_INCREMENT*(self.encoderL - self.encoderR)
                elif(self.encoderL < self.encoderR):
                    #self.yawCorrection -= self.YAW_INCREMENT
                    self.yawCorrection = self.YAW_INCREMENT*(self.encoderL - self.encoderR)
                #else:
                    #yawCorrection = 0
                #logEvent(LVL_DEBUGGING, "yawCorrection: "+str(yawCorrection))
            if(self.encoderL >= self.leftEncoderTarget) :
                self.pl = 0
                self.yawCorrection = 0
            if(self.encoderR >= self.rightEncoderTarget):
                self.pr = 0
                self.yawCorrection = 0
            #self.logger.debug("pl: "+str(self.pl)+", pr: "+str(self.pr)+", yaw: "+str(self.yawCorrection))
            self.run(self.pl, self.pr, self.yawCorrection)
            if((self.encoderL >= self.leftEncoderTarget) and (self.encoderR >= self.rightEncoderTarget)):
                self.stop("reaching target")
                return
            if( (self.directionL == self.FWD) and (self.directionR == self.FWD) and (self.distance < self.STOP_DISTANCE) ):
                self.stop("obstacle at "+str(self.distance))
                return
        else:
            self.stop("timeout")
                    
    def stop(self, reason):
        self.run(0, 0, 0)
        self.logger.info("Stopped by "+reason)
        self.freerun()

    # low level abstraction
    # l, r: PWMs (direction, middle point and offset are corrected here)
    # commands can be positive or negative here
    def run(self, l, r, yaw):
        self.pwmL = self.MIDDLE + self.OFFSETS[self.LEFT] + (l - yaw)
        self.pwmR = self.MIDDLE - self.OFFSETS[self.RIGHT] - (r + yaw)
        self.pwm.set_pwm(self.LEFT, 0, self.pwmL)
        self.pwm.set_pwm(self.RIGHT, 0, self.pwmR)
        #self.logger.debug("RUN: PWMs= "+str(self.pwmL)+", "+str(self.pwmR)+"; pl: "+str(l)+", pr: "+str(r))
        self.logger.debug("RUN -- Encod_l: %d; Encod_r: %d; Speed_l: %d; Speed_r: %d; PWM_L: %d; PWM_R: %d; PWR_L: %d; PWR_R: %d; Yaw: %d", self.encoderL, self.encoderR, self.speedL, self.speedR, self.pwmL, self.pwmR, l, r, yaw)
        
    def head(self, pan, tilt):
        self.pwm.set_pwm(self.PAN, 0, self.MIDDLE + self.OFFSETS[self.PAN] + pan)
        self.pwm.set_pwm(self.TILT, 0, self.MIDDLE + self.OFFSETS[self.TILT] + tilt)
        self.logger.info("LOOK -- Pan= "+str(pan)+" Tilt: "+str(tilt))

    def freerun(self):
        self.pwm.set_pwm(self.LEFT, 0, 0)
        self.pwm.set_pwm(self.RIGHT, 0, 0)
        self.pwm.set_pwm(self.PAN, 0, 0)
        self.pwm.set_pwm(self.TILT, 0, 0)

    def resetEncoder(self):
        self.logger.info("Reset encoder")
        self.ser.flushInput()
        while(not self.readEncoder()):
            pass
        self.leftEncoderZero += self.encoderL
        self.rightEncoderZero += self.encoderR
        while(not self.readEncoder()):
            pass

    def getData(self):
        while(not self.readEncoder()):
            pass
        return json.dumps(self.RT_data)

