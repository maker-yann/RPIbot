#!/usr/bin/env python

import time
import logging
import picamera
import picamera.array
import cv2

logger = logging.getLogger(__name__)
logger.setLevel(logging.INFO)

class Vision():

    def __init__(self):
        self.camera = picamera.PiCamera(resolution=(640, 480))
        self.camera.rotation = 180
        self.camera.start_preview()
        # Camera warm-up time
        time.sleep(2)
        
        self.image = None
        
    def close(self):
        self.camera.close()
        self.camera = None
        
    def capture(self):
        rawCapture = picamera.array.PiRGBArray(self.camera)
        self.camera.capture(rawCapture, format="bgr")
        self.image = rawCapture.array
        
    def snapshot(self):
        self.camera.capture('foo.jpg')
        
    def process(self):
        gray = cv2.cvtColor(self.image, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (3, 3), 0)
        output = cv2.Canny(blurred, 10, 200)
        self.saveImage(output, "test.jpg")

    def saveImage(self, output, filename):
        cv2.imwrite(filename, output)

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
        logger.info("Started image processing")
        v = Vision()
        v.capture()
        v.process()
        v.close()
        logger.debug("camera closed")
    except KeyboardInterrupt:
        # Signal termination 
        logger.info("Keyboard interrupt. Terminate thread")
    finally:
        logger.debug("Image processing finished")

