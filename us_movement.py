#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Mar 17 14:42:31 2021

@author: root
"""
from LIBRARY.rpi_car import rpi_movement
from LIBRARY.rpi_telemetry import mb_telemetry
from LIBRARY.rpi_movement import random_mvmnt

import numpy as np
import matplotlib.pyplot as plt
from time import time
        
        
def data_to_arrays(telemetry):
    g = -9.81
    w = np.array([[telemetry.gyrox,telemetry.gyroy, telemetry.gyroz]])
    f = np.array([[telemetry.accx,telemetry.accy, telemetry.accz]]) * g
    m = np.array([[telemetry.magx,telemetry.magy, telemetry.magz]])
    return f,w,m  
            


        
car = rpi_movement()
car.init()
mb = mb_telemetry()
mb.init_all()
mb.telemetry()
    
mvmnt = random_mvmnt(car, mb)
mvmnt.US_start()



ds = np.array([[0., 0.]])
ts = np.array([[0]])
#d = back.get_distance()
t = .0
dt = time()

while(1):
    try:
        dt = time() - dt
        t += dt
        dt = time()
  
        d = np.array([mvmnt.USs_out[l] for l in mvmnt.USs_labels])
        ds = np.append(ds, d.reshape(1,2), axis = 0)
        ts = np.append(ts, np.array([[t]]), axis = 0)
#        sleep(0.05)
        try: 
            state = mvmnt.random_mvmnt_obstacle_avoiding()
            
        except Exception:
            pass
    except KeyboardInterrupt:
        break


mvmnt.stop_mvmnt(car)
plt.plot(ts,ds)
plt.grid() 