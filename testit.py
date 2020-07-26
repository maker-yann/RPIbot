#!/usr/bin/env python

import logging
import motioncontrol
import time

def main():
    logging.basicConfig(filename='myapp.log', level=logging.INFO, format='%(asctime)s - %(levelname)s:\t%(message)s', datefmt='%Y/%m/%d %H:%M:%S')
    logging.info('Started')

    m = motioncontrol.motioncontrol()
    m.head(0, 0)
    time.sleep(1)

    #m.resetEncoder()
    #m.control(100, 100, 20, 20, 6)
    #time.sleep(1)
    while(True):
        try:
            print(m.getData()+"\r")
        except:
            break

    m.close()
    logging.info('Finished')

if __name__ == '__main__':
    main()
