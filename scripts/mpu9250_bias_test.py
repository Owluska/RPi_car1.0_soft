#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Mar 23 11:20:56 2021

@author: root
"""

from LIBRARY.mpu9250_i2c import mpu6050_conv, AK8963_conv
import csv
import numpy as np
import matplotlib.pyplot as plt
from time import time

cal_file = 'calibration_data/mpu9250_cal_params.csv'

np.set_printoptions(precision=2)
np.set_printoptions(suppress=True)

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

def mpu_calibrated(file):
    offsets = read_calibration_file(file)
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


def plot_fig(data, label):
    fig = plt.figure(figsize = (10,10))
    plt.title(label)
    plt.plot(data)
    plt.grid()    

t = .0
dt = time()
data = mpu_calibrated(cal_file).reshape(1,9)
while(t<3600):
    dt = time() - dt
    t += dt
    dt = time()
    try:
        vector = mpu_calibrated(cal_file)
        data = np.append(data, vector.reshape(1,9), axis = 0)
    except KeyboardInterrupt:
        break
    print("time: {:.1f}s, data: {}".format(t, vector))

plot_fig(data[:, 0:3], 'ACC')
plot_fig(data[:, 3:6], 'W') 
plot_fig(data[:, 6:9], 'M')                    