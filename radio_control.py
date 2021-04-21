#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Apr 20 11:59:56 2021

@author: root
"""
#import serial
#from time import sleep
#
#port = serial.Serial("/dev/ttyAMA0", baudrate = 9600)
#while(True):
#    try:
#        port.write(b'raspppp\n')
#        print('sent: raspppp')
#        sleep(2)
#    except KeyboardInterrupt:
#        break
#port.close()


import wiringpi
from time import sleep

wiringpi.wiringPiSetup()


port = wiringpi.serialOpen("/dev/ttyAMA0", 9600)
while(True):
    try:
        wiringpi.serialPuts(port, 'rasp\n')
        out = ''
        while wiringpi.serialDataAvail(port) > 0:
            
            out += wiringpi.serialGetchar(port)
        #print('sent: raspppp')
        sleep(2)
    except KeyboardInterrupt:
        break
wiringpi.serialClose(port)