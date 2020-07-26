#!/usr/bin/env python

import logging
import motioncontrol
from tornado.options import options, define, parse_command_line
import tornado.httpserver
import tornado.ioloop
import tornado.web
import tornado.wsgi
import tornado.websocket
import json
import time
import Queue
import os.path
import signal
import threading

define('port', type=int, default=8080)

class Application(tornado.web.Application):
    def __init__(self):
        handlers = [(r"/", webServerHandler), (r"/websocket", MyWebSocket)]
        settings = dict(
            #template_path=os.path.join(os.path.dirname(__file__), "templates"),
            static_path=os.path.join(os.path.dirname(__file__), "static"),
        )
        super(Application, self).__init__(handlers, **settings)

class webServerHandler(tornado.web.RequestHandler):
    def get(self):
        # render the html page, the socket is related to different domain
        self.render("gui.html")

class MyWebSocket(tornado.websocket.WebSocketHandler):
    locked = False
    def check_origin(self, origin):
        return True

    def myTimer(self):
        self.locked = True
        text = ""
        try:
            while(True):
                t = q_Data.get(False)
                text = t
        except Queue.Empty:
            pass
        try:
            if text:
                self.write_message(text)
        except:
            self.locked = False
            return
        self.timer = threading.Timer(0.5, self.myTimer)
        self.timer.start()
        self.locked = False

    def open(self):
        logger.info("WebSocket opened")
        self.timer = threading.Timer(0.5, self.myTimer)
        self.timer.start()

    def on_message(self, message):
        #logger.debug("Server WS msg received: "+ message)
        try:
            self.write_message(message)
        except:
            logger.warning("Connection was closed in on_message")
        q_Command.put(message)
        if(message=="exit"):
            while(self.locked):
                pass
            self.timer.cancel()
            self.close()

    def on_close(self):
        logger.info("WebSocket closed")

def controlThread():
    logger.debug("Control thread started")
    msg = ""

    m = motioncontrol.motioncontrol()
    m.head(0, 0)
    m.resetEncoder()
    time.sleep(1)
    cmd = 0

    while(msg != "exit"):
        q_Data.put(m.getData())
        # read message from frontend
        try:
            msg = q_Command.get(False)
        except Queue.Empty:
            if(cmd == 1):
                m.cyclicMove()
        else:
            logger.info("Server received command: "+msg)
            data = msg.split(';')
            if(data[0] == "HEAD"):
                m.head(int(data[1]), int(data[2]))
            elif(data[0] == "MOVE"):
                m.resetEncoder()
                m.startMove(int(data[1]), int(data[2]), int(data[3]), int(data[4]),int(data[5]))
                cmd = 1
                #m.control(100, 100, 20, 20, 6)
            elif(data[0] == "STOP"):
                cmd = 0
                m.stop("driver request")
        time.sleep(0.01)
    logger.debug("Control thread ended")
    # stop server
    tornado.ioloop.IOLoop.instance().stop()

def main():
    interrupted = False
    tornado.options.parse_command_line()
    app = Application()
    app.listen(options.port)

    try:
        tornado.ioloop.IOLoop.instance().start()
        logger.debug("Tornado loop start() finished")
    except KeyboardInterrupt:
        tornado.ioloop.IOLoop.instance().stop()
        q_Command.put("exit")
        logger.debug("User interrupt")
        interrupted = True
    logger.debug("Main terminated with interrupt = " + str(interrupted))
    return interrupted

# Queue from html WS server to motioncontrol
q_Command = Queue.Queue()
# Queue from motioncontrol to WS html server
q_Data = Queue.Queue()

if __name__ == '__main__':

    # create logger with 'spam_application'
    logger = logging.getLogger(__name__)
    logger.setLevel(logging.DEBUG)
    # create file handler which logs even debug messages
    fh = logging.FileHandler('rpibot.log')
    fh.setLevel(logging.DEBUG)
    # create console handler with a higher log level
    #ch = logging.StreamHandler()
    #ch.setLevel(logging.DEBUG)
    # create formatter and add it to the handlers
    formatter = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s', datefmt='%Y/%m/%d %H:%M:%S')
    fh.setFormatter(formatter)
    #ch.setFormatter(formatter)
    # add the handlers to the logger
    logger.addHandler(fh)
    #logger.addHandler(ch)

    logger.info('Started info')
    logger.debug('Started debug')

    try:
        while(True):
            logger.debug("Starting control thread")
            t = threading.Thread(target=controlThread, args=( ))
            t.start()
            interrupted = main()
            logger.debug("Main loop finished")
            t.join()
            if(interrupted):
                break
    except KeyboardInterrupt:
        logger.debug("User interrupt")
    logger.debug("terminated")

