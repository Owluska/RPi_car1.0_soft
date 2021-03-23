#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Mar 17 14:42:31 2021

@author: root
"""
from LIBRARY.rpi_car import rpi_movement
from LIBRARY.rpi_telemetry import mb_telemetry
from LIBRARY.rpi_movement import random_mvmnt
from LIBRARY.mpu9250_i2c import mpu6050_conv, AK8963_conv

import numpy as np
import matplotlib.pyplot as plt
from time import time
import csv




def read_calibration_file(cal_filename):
    cal_offsets = np.array([[],[],[],0.0,0.0,0.0,[],[],[]], dtype = 'object')
    with open(cal_filename,'r',newline='') as csvfile:
        reader = csv.reader(csvfile,delimiter=',')
        iter_ii = 0
        for row in reader:
            if len(row)>2:
                row_vals = [float(ii) for ii in row[int((len(row)/2)+1):]]
                cal_offsets[iter_ii] = row_vals
            else:
                cal_offsets[iter_ii] = float(row[1])
            iter_ii+=1
    return cal_offsets

#
def mpu_get_data():
    ax,ay,az,wx,wy,wz = mpu6050_conv() # read and convert mpu6050 data
    mx,my,mz = AK8963_conv() 
    return np.array([ax,ay,az,wx,wy,wz,mx,my,mz])

def mpu_calibrated(cal_file):
    offsets = read_calibration_file(cal_file)
    mpu_raw = mpu_get_data()
    mpu_cal = np.zeros_like(mpu_raw)
    cal_rot_indicies = [[6,7],[7,8],[6,8]] # heading indices
    for i in range(3):   
        mpu_cal[i] = offsets[i][0]*mpu_raw[i]+ offsets[i][1]
    for i in range(3,6):
        mpu_cal[i] = mpu_raw[i] - offsets[i]
    for i in range(6,9):
        j = i-6
        mpu_cal[i] = mpu_raw[i] - offsets[cal_rot_indicies[j][0]]
    return mpu_cal        
        
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

cal_file = 'calibration_data/mpu9250_cal_params.csv'
data = mpu_calibrated(cal_file).reshape(1,9)
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
        
        vector = mpu_calibrated(cal_file)
        data = np.append(data, vector.reshape(1,9), axis = 0)
#        sleep(0.05)
        try: 
            state = mvmnt.random_mvmnt_obstacle_avoiding()
            
        except Exception:
            pass
    except KeyboardInterrupt:
        break


mvmnt.stop_mvmnt(car)
#plt.plot(ts,ds)
#plt.grid() 
plot_fig(data[:, 0:3], 'ACC')
plot_fig(data[:, 3:6], 'W') 
plot_fig(data[:, 6:9], 'M')  