#!/usr/bin/env python

import time
import logging
import Sense, Control, Vision

from tornado.options import options, define, parse_command_line
from tornado.ioloop import PeriodicCallback
import tornado.httpserver
import tornado.ioloop
import tornado.web
import tornado.wsgi
import tornado.websocket
import json
import os.path
import signal
import base64
import hashlib
import picamera
import picamera.array
import cv2
try:
    import cStringIO as io
except ImportError:
    import io

logger = logging.getLogger(__name__)
logger.setLevel(logging.INFO)
# create file handler which logs even debug messages
#fh = logging.FileHandler('rpibot.log')
#fh.setLevel(logging.DEBUG)
# create console handler with a higher log level
##ch = logging.StreamHandler()
##ch.setLevel(logging.DEBUG)
# create formatter and add it to the handlers
##formatter = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s', datefmt='%Y/%m/%d %H:%M:%S')
#fh.setFormatter(formatter)
##ch.setFormatter(formatter)
# add the handlers to the logger
##logger.addHandler(fh)
##logger.addHandler(ch)


define('port', type=int, default=8080)

class Application(tornado.web.Application):
    def __init__(self):
        handlers = [(r"/", webServerHandler), (r"/websocket", MyWebSocket)]
        settings = dict(
            static_path=os.path.join(os.path.dirname(__file__), "static"),
        )
        super(Application, self).__init__(handlers, **settings)

class webServerHandler(tornado.web.RequestHandler):
    def get(self):
        # render the html page, the socket is related to different domain
        self.render("gui.html")

class MyWebSocket(tornado.websocket.WebSocketHandler):
    
    camera = None
    camera_loop = None
    
    def check_origin(self, origin):
        return True

    def guiLoop(self):
        self.write_message(json.dumps(s.RT_data))

    def open(self):
        logger.info("WebSocket opened")
        self.camera = picamera.PiCamera(resolution=(640, 480))
        self.camera.rotation = 180
        self.camera.start_preview()
        # Camera warm-up time
        time.sleep(2)
        self.visio = Vision.Vision(cam=self.camera)
        self.gui_loop = PeriodicCallback(self.guiLoop, 500)
        self.gui_loop.start()

    def on_message(self, message):
        logger.debug("Server WS msg received: "+ message)
        if(message=="exit"):
            self.timer.cancel()
            self.close()
        elif(message=="video;on"):
            self.camera_loop = PeriodicCallback(self.cameraLoop, 1000)
            self.camera_loop.start()
        elif(message=="video;off"):
            self.camera_loop.stop()
        elif(message=="pic"):
            self.visio.capture()
            self.visio.process()
        else:
            c.runCommand(message)

    def cameraLoop(self):
        """Sends camera images in an infinite loop."""
        #sio = io.StringIO()
        sio = io.BytesIO()
        self.camera.capture(sio, "jpeg", use_video_port=True)
        try:
            self.write_message(base64.b64encode(sio.getvalue()))
        except tornado.websocket.WebSocketClosedError:
            self.camera_loop.stop()

    def on_close(self):
        logger.info("WebSocket closed")
        try:
            if(self.camera_loop != None):
                self.camera_loop.stop()
            logger.debug("camera loop stopped")
            time.sleep(1)
            self.gui_loop.stop()
            logger.debug("gui loop stopped")
            time.sleep(1)
            self.visio.close()
            self.camera = None
            logger.debug("camera closed")
        except:
            raise


def main():
    interrupted = False
    tornado.options.parse_command_line() # this is calling the tornado logging settings
    app = Application()
    app.listen(options.port)

    try:
        tornado.ioloop.IOLoop.instance().start()
        logger.debug("Tornado loop start() finished")
    except KeyboardInterrupt:
        tornado.ioloop.IOLoop.instance().stop()
        #q_Command.put("exit")
        logger.debug("User interrupt (main)")
        interrupted = True
    logger.debug("Main terminated with interrupt = " + str(interrupted))
    return interrupted

# This is the main application to be called to run the whole robot
if __name__ == '__main__':

    logger.debug("Starting control thread")
    logger.debug("Starting senor thread")
    s = Sense.Sense()
    s.start()
    c = Control.Control(s)
    c.start()
    while(1):
        logger.info("Starting web server")
        interrupted = main()
        if(interrupted == True):
            break
    # Signal termination
    logger.info("User interrupt")
    s.terminate()
    c.terminate()
    logger.debug("Main loop finished (__main__")
    # Wait for actual termination (if needed)
    c.join()
    s.join()
    logger.info("terminated")

