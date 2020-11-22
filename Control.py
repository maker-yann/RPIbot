#!/usr/bin/env python

import time
import logging
try:
    import ConfigParser as configparser
except:
    import configparser
from threading import Thread
import Adafruit_PCA9685
import RPi.GPIO as GPIO
import Sense

logger = logging.getLogger(__name__)
logger.setLevel(logging.INFO)
# create console handler with a higher log level
##ch = logging.StreamHandler()
##ch.setLevel(logging.DEBUG)
# create formatter and add it to the handlers
##formatter = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s', datefmt='%Y/%m/%d %H:%M:%S')
##ch.setFormatter(formatter)
# add the handlers to the logger
##logger.addHandler(ch)

def bound(value, mini, maxi):
    if value > maxi:
        return maxi
    elif value < mini:
        return mini
    else:
        return value

class Actuation():
    SERVO = 0
    DC    = 1
    WHEELS = DC

    LEFT  = 0
    RIGHT = 1
    PAN   = 2
    TILT  = 3

    STP  =  0
    RWD  = -1
    FWD  =  1

    OFFSETS = [0, 0, 0, 0]    # overwritten by ini file settings
    MIDDLE = 400              # overwritten by ini file settings
    FRICTION_OFFSET = 10

    # Right H-bridge
    #ENA = 12
    GPIO_IN1 = 12 # instead of 6 as pin is pulled low
    GPIO_IN2 = 13

    # Left H-bridge
    #ENB = 20
    GPIO_IN3 = 19
    GPIO_IN4 = 26

    PWM_OFFSET_L = 35
    PWM_OFFSET_R = 33#35

    def __init__(self, sensors):
        self.pwm = Adafruit_PCA9685.PCA9685()
        self.pwm.set_pwm_freq(60)
        self.pwmL = 0
        self.pwmR = 0
        self.lastDirectionL = self.STP
        self.lastDirectionR = self.STP
        self.motor_L = None
        self.motor_R = None
        self.sensors = sensors

        self.readIni()
        logger.debug("Read MIDDLE from ini file: "+str(self.MIDDLE))
        self.initMotors()

    # low level abstraction
    # l, r: PWMs (direction, middle point and offset are corrected here)
    # commands can be positive or negative here
    def actuateWheels(self, l, r, dirL, dirR):
        if self.WHEELS == self.SERVO:
            # Compensate friction around middle servo position
            if l>0:
                self.pwmL = self.MIDDLE + self.OFFSETS[self.LEFT] + l
            else:
                self.pwmL = self.MIDDLE + self.OFFSETS[self.LEFT] + l
            if r>0:
                self.pwmR = self.MIDDLE - self.OFFSETS[self.RIGHT] - r
            else:
                self.pwmR = self.MIDDLE - self.OFFSETS[self.RIGHT] - r
            self.pwm.set_pwm(self.LEFT, 0, self.pwmL)
            self.pwm.set_pwm(self.RIGHT, 0, self.pwmR)
        else:
            if dirL != self.lastDirectionL:
                self.motor_L.stop()
                self.pwmL = 0
                if dirL == self.FWD:
                    self.motor_L = GPIO.PWM(self.GPIO_IN3, 100)
                    self.motor_L.start(0)
                elif dirL == self.RWD:
                    self.motor_L = GPIO.PWM(self.GPIO_IN4, 100)
                    self.motor_L.start(0)
                    l = -l
            self.lastDirectionL = dirL
            if dirL != self.STP:
                self.pwmL = self.PWM_OFFSET_L + l
                self.pwmL = bound(self.pwmL, 0, 100)
                self.motor_L.ChangeDutyCycle(self.pwmL)

            if dirR != self.lastDirectionR:
                self.motor_R.stop()
                self.pwmR = 0
                if dirR == self.FWD:
                    self.motor_R = GPIO.PWM(self.GPIO_IN1, 100)
                    self.motor_R.start(0)
                elif dirR == self.RWD:
                    self.motor_R = GPIO.PWM(self.GPIO_IN2, 100)
                    self.motor_R.start(0)
                    r = -r
            self.lastDirectionR = dirR
            if dirR != self.STP:
                self.pwmR = self.PWM_OFFSET_R + r
                self.pwmR = bound(self.pwmR, 0, 100)
                self.motor_R.ChangeDutyCycle(self.pwmR)
            self.sensors.RT_data["pwmL"] = self.pwmL
            self.sensors.RT_data["pwmR"] = self.pwmR

            #logger.debug("ACTUATION -- PWM_L: %d; PWM_R: %d; PWR_L: %d; PWR_R: %d;dirL: %d; lastDirL: %d", self.pwmL, self.pwmR, l, r, dirL, self.lastDirectionL)

    def actuateHead(self, pan, tilt):
        self.pwm.set_pwm(self.PAN, 0, self.MIDDLE + self.OFFSETS[self.PAN] + pan)
        self.pwm.set_pwm(self.TILT, 0, self.MIDDLE + self.OFFSETS[self.TILT] + tilt)
        logger.info("LOOK -- Pan= "+str(pan)+" Tilt: "+str(tilt))

    def initMotors(self):
        #GPIO.setup(ENA, GPIO.OUT)
        GPIO.setup(self.GPIO_IN1, GPIO.OUT)
        GPIO.setup(self.GPIO_IN2, GPIO.OUT)

        #GPIO.setup(ENB, GPIO.OUT)
        GPIO.setup(self.GPIO_IN3, GPIO.OUT)
        GPIO.setup(self.GPIO_IN4, GPIO.OUT)

        GPIO.output(self.GPIO_IN1, 0)
        GPIO.output(self.GPIO_IN2, 0)
        GPIO.output(self.GPIO_IN3, 0)
        GPIO.output(self.GPIO_IN4, 0)

        self.motor_L = GPIO.PWM(self.GPIO_IN3, 100)  # frequency=100Hz
        self.motor_R = GPIO.PWM(self.GPIO_IN1, 100)  # frequency=100Hz

    def freerun(self):
        self.pwm.set_pwm(self.LEFT, 0, 0)
        self.pwm.set_pwm(self.RIGHT, 0, 0)
        self.pwm.set_pwm(self.PAN, 0, 0)
        self.pwm.set_pwm(self.TILT, 0, 0)

    def readIni(self):
        logger.info("Read ini file")
        #Config = ConfigParser.ConfigParser()
        Config = configparser.ConfigParser()
        Config.read("config.ini")
        self.OFFSETS[self.LEFT] = Config.getint('motor', 'offset_left')
        self.OFFSETS[self.RIGHT] = Config.getint('motor', 'offset_right')
        self.OFFSETS[self.PAN] = Config.getint('motor', 'offset_pan')
        self.OFFSETS[self.TILT] = Config.getint('motor', 'offset_tilt')
        self.MIDDLE = Config.getint('motor', 'middle')

    def close(self):
        self.freerun()
        self.motor_L.stop()
        self.motor_R.stop()
        GPIO.cleanup()

class Control(Thread):

    TRACE = 1

    TURN =  0
    STP  =  0
    RWD  = -1
    FWD  =  1

    STOP_DISTANCE = 10
    OPEN_LOOP_PWM = 20
    SPEED_HYSTERESIS = 15
    #PWM_INCREMENT = 2

    def __init__(self, sensors):
        Thread.__init__(self)
        self.tracefile = None
        self.traceline = 0
        self.traceData = {}
        self.directionL = self.STP
        self.directionR = self.STP
        self.initialYaw = 0
        self.initialEncoderL = 0
        self.initialEncoderR = 0
        self.yawCorrection = 0
        self.odo_distance = 0
        self.odo_rotation = 0
        self.I_L = 0
        self.I_R = 0
        self.Iyaw = 0
        self.parameters = {}
        self.moving = False
        self.sensors = sensors
        self.lastFrameTimestamp = time.time()
        self.cycleTime = 0
        self._running = True

        self.initParameters()
        self.actuation = Actuation(sensors)
        self.actuation.actuateHead(0, 0)
        time.sleep(0.5)
        self.actuation.freerun()

### Initialize external tunable parameters
    def initParameters(self):
        self.parameters["Kp"] = 0.6
        self.parameters["Ki"] = 0.1
        self.parameters["Kp_yaw"] = 2.0
        self.parameters["Ki_yaw"] = 0.0 # producing oscillations
        self.parameters["PWMinc"] = 2

    def initTrace(self):
        if(self.TRACE == 1):
            self.tracefile = open("trace.csv","w+")
            header = "timestamp;"
            #for k, v in self.traceData.iteritems():
            for k, v in self.traceData.items():
                header += k+";"
            header += "\r\n"
            self.tracefile.write(header)
            
    def writeTrace(self):
        if (self.TRACE == 1):
            if self.traceline == 0:
                self.initTrace()
                self.traceline = 1
            filestring = str(time.time())+";"
            displaystring = ""
            #for k, v in self.traceData.iteritems():
            for k, v in self.traceData.items():
                displaystring += k+": "+str(v)+", "
                filestring += str(v)+";"
            filestring += "\r\n"
            #logger.debug(displaystring)
            self.tracefile.write(filestring)

    def terminate(self): 
        self._running = False
        
### This function is not yet used. It will probably contain the image processing
    def idleTask(self):
        pass

    def close(self):
        logger.debug("Closing control thread")
        self.actuation.actuateHead(0, 0)
        time.sleep(1)
        self.actuation.close()
        if self.TRACE == 1 and self.tracefile != None:
            self.tracefile.close()

    def runCommand(self, cmd):
        logger.debug("Control thread received command: " + cmd)
        data = cmd.split(';')
        if(data[0] == "HEAD"):
            self.actuation.actuateHead(int(data[1]), int(data[2]))
        elif(data[0] == "MOVE"):
            self.prepareMove(int(float(data[1])), int(float(data[2])), int(data[3]), int(data[4]),int(data[5]))
            self.moving = True
        elif(data[0] == "STOP"):
            self.stop("driver request")
        elif(data[0] == "SET"):
            try:
                self.parameters[data[1]] = float(data[2])
                logger.info("Parameter " + data[1] + " set to: " + data[2])
            except:
                logger.warning("Parameter not defined: " + data[1])

    def prepareMove(self, leftSpeedTarget, rightSpeedTarget, leftEncoderTarget, rightEncoderTarget, timeout):
        # Store targets
        self.leftSpeedTarget = leftSpeedTarget
        self.rightSpeedTarget = rightSpeedTarget
        self.timeout = timeout
        self.t = time.time()
        self.pr = 0
        self.pl = 0

        logger.info("Calling MOVE with left speed = "+str(leftSpeedTarget)+", right speed = "+str(rightSpeedTarget)+", left encoder = "+str(leftEncoderTarget)+", right encoder = "+str(rightEncoderTarget))
        # Calculate wheel directions
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
        self.leftEncoderTarget = self.sensors.RT_data["encoderL"] + leftEncoderTarget * self.directionL # target calculation depends on direction
        self.rightEncoderTarget = self.sensors.RT_data["encoderR"] + rightEncoderTarget * self.directionR
        self.yawCorrection = 0
        self.initialEncoderL = self.sensors.RT_data["encoderL"]
        self.initialEncoderR = self.sensors.RT_data["encoderR"]
        self.initialYaw = self.sensors.RT_data["yaw"]
        self.sensors.directionL = self.directionL # centralize sense information in Sense module (to be able to calculate odometer)
        self.sensors.directionR = self.directionR
        self.lastFrameTimestamp = time.time()

    def openLoopControl(self):
        if((time.time() - self.t) < self.timeout):
            if not self.checkTargetReached():
                # Todo: to be changed wheel and direction individual
                self.pl = self.OPEN_LOOP_PWM
                self.pr = self.OPEN_LOOP_PWM

                self.actuation.actuateWheels(self.pl, self.pr, self.directionL, self.directionR)

                self.traceData = self.sensors.RT_data.copy()
                self.traceData["CycleTime"] = self.cycleTime
                self.writeTrace()
        else:
            self.stop("timeout")

    def closeLoopControl(self):
        if((time.time() - self.t) < self.timeout):
            if not self.checkTargetReached():
                # Yaw control first to correct speed target (positive clockwise) - Yaw correction only for straight movement
                eYaw = self.sensors.RT_data["yaw"] - self.initialYaw
                if self.actuation.pwmL<100 and self.actuation.pwmR<100:
                    self.Iyaw += eYaw
                if self.directionL == self.FWD and self.directionR == self.FWD:
                    self.yawCorrection = int(self.parameters["Kp_yaw"] * eYaw + self.parameters["Ki_yaw"] * self.Iyaw)
                elif self.directionL == self.RWD and self.directionR == self.RWD:
                    self.yawCorrection = -(int(self.parameters["Kp_yaw"] * eYaw + self.parameters["Ki_yaw"] * self.Iyaw))
                else:
                    self.yawCorrection = 0
                    self.Iyaw = 0

                # Speed control left
                eSpeedL = abs(self.leftSpeedTarget) - int(self.sensors.RT_data["speedL"]) - self.yawCorrection
                self.I_L += eSpeedL
                if eSpeedL < 0:
                    eSpeedL = 0
                self.pl = int(self.parameters["Kp"]*eSpeedL+self.parameters["Ki"]*self.I_L)
                # Speed control right
                eSpeedR = abs(self.rightSpeedTarget) - int(self.sensors.RT_data["speedR"]) + self.yawCorrection
                self.I_R += eSpeedR
                if eSpeedR < 0:
                    eSpeedR = 0
                self.pr = int(self.parameters["Kp"]*eSpeedR+self.parameters["Ki"]*self.I_R)

                # This is the output for actuation
                self.actuation.actuateWheels(self.pl, self.pr, self.directionL, self.directionR)

                # Here we copy some data to make it available in the trace/plot
                self.cycleTime = int(round((time.time() - self.lastFrameTimestamp)*1000))
                self.lastFrameTimestamp = time.time()
                self.traceData = self.sensors.RT_data.copy()
                self.traceData["CycleTimeControl"] = self.cycleTime
                self.traceData["I_L"] = self.I_L
                self.traceData["I_R"] = self.I_R
                self.traceData["eYaw"] = eYaw
                self.traceData["Iyaw"] = self.Iyaw
                self.traceData["eSpeedL"] = eSpeedL
                self.traceData["eSpeedR"] = eSpeedR
                self.traceData["yawCorr"] = self.yawCorrection
                self.writeTrace()
        else:
            self.stop("timeout")
            self.I_L = 0
            self.I_R = 0
            self.Iyaw = 0

    def checkTargetReached(self):
        res = False

        if ((self.sensors.RT_data["encoderL"] >= self.leftEncoderTarget) and 
            (self.sensors.RT_data["encoderR"] >= self.rightEncoderTarget) and 
            (self.directionL == self.FWD) and (self.directionR == self.FWD)):
            self.stop("reaching target")
            res = True
        elif ((self.sensors.RT_data["encoderL"] <= self.leftEncoderTarget) and 
            (self.sensors.RT_data["encoderR"] <= self.rightEncoderTarget) and 
            (self.directionL == self.RWD) and (self.directionR == self.RWD)):
            self.stop("reaching target")
            res = True
#        elif( (self.directionL == self.FWD) and (self.directionR == self.FWD) and (self.sensors.RT_data["distance"] < self.STOP_DISTANCE) ):
#            self.stop("obstacle at "+str(self.sensors.RT_data["distance"]))
#            res = True
        return res

    def run(self):
        logger.debug('Control thread running')
        while self._running: 
            time.sleep(0.05)
            ### Wait for command (call of runCommand by rpibot.py)
            if(self.moving == True):
                self.closeLoopControl()
                #self.openLoopControl()
            else:
                self.idleTask()
        self.close() 
        logger.debug('Control thread terminating')

    def stop(self, reason):
        self.moving = False
        self.pl = 0
        self.pr = 0
        self.yawCorrection = 0
        self.I_L = 0
        self.I_R = 0
        self.Iyaw = 0
        self.actuation.actuateWheels(0, 0, self.STP, self.STP)
        logger.info("Stopped by "+reason)
        self.actuation.freerun()

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
        s = Sense.Sense()
        s.start()
        c = Control(s) 
        c.start()
        c.actuation.actuateHead(-20, -20)
        time.sleep(1)
        c.actuation.actuateHead(20, 20)
        time.sleep(1)
        c.actuation.actuateWheels(50, 50, 1, 1)
        time.sleep(2)
        c.stop("Test")
    except KeyboardInterrupt:
        # Signal termination 
        logger.info("Keyboard interrupt. Terminate thread")
    finally:
        c.terminate()
        s.terminate()
        logger.debug("Thread terminated")

        # Wait for actual termination (if needed)  
        c.join()
        s.join()
        logger.debug("Thread finished")

