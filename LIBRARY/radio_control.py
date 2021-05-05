#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Apr 20 11:59:56 2021

@author: root
"""
import serial
from time import sleep



#remote = {'FORWARD_LEFT':'1', 'FORWARD':'2','FORWARD_RIGHT':'3',
#          'LEFT':'4', 'STOP':'5','RIGHT':'6','BACKWARD_LEFT':'7',
#          'BACKWARD':'8','BACKWARD_RIGHT':'9','TURN_LEFT':'A','TURN_RIGHT':'B'}


remote = {'FORWARD':'2','STOP':'5','BACKWARD':'8','TURN_LEFT':'A','TURN_RIGHT':'B'}

state = 'STOP'
port = serial.Serial("/dev/ttyS0", baudrate = 9600)
while(True):
    try:
        s = ""
        encoding = 'utf-8'
        if port.in_waiting > 0:
            read = port.read()
            s = read.decode(encoding)
        if s not in remote.values():
            continue
        else:
            state = str([k for k,v in remote.items() if v == s][0])
            print(state)
            toWrite = bytes(state, encoding)
            port.write(toWrite)
    except KeyboardInterrupt:
        break
port.close()

