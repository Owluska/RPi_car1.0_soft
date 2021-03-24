#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Mar 17 14:42:31 2021

@author: root
"""
from LIBRARY.rpi_car import rpi_movement
from LIBRARY.rpi_telemetry import mb_telemetry
from LIBRARY.rpi_US_multi import US_multi

import numpy as np
import matplotlib.pyplot as plt
from time import time
       
        
def data_to_arrays(telemetry):
    w = np.array([[telemetry.gyrox,telemetry.gyroy, telemetry.gyroz]]) * mb.D2R
    f = np.array([[telemetry.accx,telemetry.accy, telemetry.accz]]) * mb.g
    m = np.array([[telemetry.magx,telemetry.magy, telemetry.magz]])
    return f,w,m  
            

def plot_fig(data, label):
    fig = plt.figure(figsize = (10,10))
    plt.title(label)
    plt.plot(data)
    plt.grid() 
        


car = rpi_movement()
car.init()
mb = mb_telemetry()
mb.init_all()
mb.telemetry()
    
uss = US_multi()
uss.US_start()

#cal_file = 'calibration_data/mpu9250_cal_params.csv'
#data = mpu_calibrated(cal_file).reshape(1,9)
ds = np.array([[0., 0.]])
ts = np.array([[0]])
#d = back.get_distance()
t = .0
dt = time()
fs,ws,ms = data_to_arrays(mb)
#car.stop()

while(1):
    try:        
        dt = time() - dt
        mb.time += dt
        dt = time()

        
        
        mb.telemetry()
        
        f,w,m = data_to_arrays(mb)

    
        d = np.array([uss.USs_out[l] for l in uss.USs_labels])
        ds = np.append(ds, d.reshape(1,2), axis = 0)
        ts = np.append(ts, np.array([[t]]), axis = 0)      
        
        ws = np.append(ws, w.reshape(1,3), axis = 0)
        fs = np.append(fs, f.reshape(1,3), axis = 0)
        ms = np.append(ms, m.reshape(1,3), axis = 0)
#        sleep(0.05)
        try: 
            car.move_backward()
            print(d)
            
        except Exception as e:
            template = "An exception of type {0} occured. Arguments:\n{1!r}"
            message = template.format(type(e).__name__, e.args)
            print(message)
            pass
    except KeyboardInterrupt:
        break


uss.USs_stop()
car.stop()
#plt.plot(ts,ds)
#plt.grid() 
plot_fig(fs, 'ACC')
plot_fig(ws, 'W') 
plot_fig(ms, 'M')  