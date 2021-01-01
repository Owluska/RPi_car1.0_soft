#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sat Dec 12 15:56:29 2020

@author: root
"""

from LIBRARY.rpi_car import rpi_movement
from LIBRARY.rpi_car_mag_calibration import write_calibration_file
from LIBRARY.rpi_telemetry import mb_telemetry
from time import time
from datetime import datetime
import math
import numpy as np
from filterpy.common import Q_discrete_white_noise
from filterpy.kalman import KalmanFilter
import matplotlib.pyplot as plt

def create_file(name = 'test', path = '/home/pi/Desktop/RPi_car1.0_soft/output/', label = ''):
    now = datetime.now()
    date_time = now.strftime("_%m_%d_%Y_%H_%M_%S")
    name += date_time
    f = open(path+name, 'a+')
    f.write(label)
    f.write('\n')
    return f

def yaw_from_mags(board):
    magx = board.magx
    magy = board.magy
    R2D = 180/math.pi
    yaw = round(math.atan2(-magy, magx) * R2D, 3)
    return yaw

#def yaw_from_gyro_rk4(gyros, telemetry):
#    dt = telemetry[0]
#    gyros.append(telemetry[5])
#    g_4, g_3, g_2, g_1 = 0,0,0,0
#    length = len(gyros)
#    if length == 0:
#        return 0
#    elif length == 1:
#        g_1 = gyros[-1]
#    elif length == 2:
#        g_1 = gyros[-1]
#        g_2 = gyros[-2]
#    elif length == 3:
#        g_1 = gyros[-1]
#        g_2 = gyros[-2]
#        g_3 = gyros[-3]
#    else:
#        g_1 = gyros[-1]
#        g_2 = gyros[-2]
#        g_3 = gyros[-3]
#        g_4 = gyros[-4]
#    yaw = g_2*dt + 1/6*(g_1 + 2*g_2 + 2*g_3 + g_4)
#    yaw = yaw % (-180)
#    return dt, g_1, round(yaw, 3)
    

def yaw_from_gyro_euler(gyros, board):
    dt = board.time
    gyros.append(board.gyroz)
    
    length = len(gyros)
    if length == 0:
        return 0
    elif length == 1:
        yaw = gyros[0] * dt
    else:
        yaw = gyros[-2] + gyros[-1]*dt

    if abs(yaw) > 180:
        yaw = -180 + yaw % (360)
    yaw = round(yaw,3)
    return yaw, gyros


def complementary_filter(data1, data2, alpha = 1):
    output = data1 * alpha + data2 * (1 - alpha)
    output = round(output, 3)
    return output

def kalman_filter_init(board, yaw = 0, gyro = 0):
    tracker = KalmanFilter(dim_x=2, dim_z=2)
    dt = board.time
    tracker.F = np.array([[1, dt],
                          [0,  1]])
    tracker.H = np.array([[1, 0],
                          [0, 1]])
    
    gyro_std = 0.04659871242856395
    yaw_std = 2.4556563053489393
    tracker.R = np.array([[yaw_std,          0],
                          [        0, gyro_std]])
    
    Q = Q_discrete_white_noise(dim=2, dt = dt, var = 0.01)
#    Q = np.array([[Q[0][0], Q[0][1]],
#                  [Q[1][0], Q[1][1]]])
    tracker.Q = Q
    tracker.P = np.eye(2) * 10 # +/- 10 deg
    
    z_0 = [yaw, gyro]
    tracker.z = np.array(z_0)
    tracker.x = np.array(z_0)
    return tracker

def kalman_filter_get_prediction(tracker, board, gyro_std, yaw_std):

    kf.R = np.array([[yaw_std,          0],
                      [        0, gyro_std]])
    
    kf.z = np.array([yaw_from_mags(board), board.gyroz])
    
    kf.predict()
    kf.update(kf.z)
    yaw_kf = round(kf.x[0], 3)
    
    return kf.z[0], yaw_kf
    
def plot_data_n_labels(x, ys, title = '', xlabel = '',
                       ylabel = '', legend =None):
    fig = plt.figure(figsize = (10,10))
    
    for y in ys:
        plt.plot(x,y)
    plt.title(title)
    plt.xlabel(xlabel)
    plt.ylabel(ylabel)
    
    if legend != None:
        plt.legend(legend)

car = rpi_movement()
car.init()
mb = mb_telemetry()
mb.init_all()


toCalibrate = False
attempts = 5
if toCalibrate:
  write_calibration_file(car, auto = False, attempts = attempts)
  

mb.read_mag_calibration_file()
print("Magnetometer caibration:")    
print(mb.magx_offset, mb.magx_scale, mb.magy_offset, mb.magy_scale)

label = "time gyroz magX magY yaw_mag yaw_gyro yaw_cf yaw_kf"
print(label)
f = create_file(name = 'filters', label = label)

dt = time()

t, gyro, magx, magy, yaw_rm, yaw_rg, yaw_cf, yaw_kf = 0.15,0,0,0,0,0,0,0
gyros = []
sdata = ''

mb.time = t
telemetry = mb.telemetry()
kf = kalman_filter_init(mb, yaw = yaw_from_mags(mb), gyro = mb.gyroz)
#print(kf)

obstacle_threshold = 15
voltage_threshold = 6.7
uv_counter = 0
gyro_std, yaw_std = 0.04659871242856395,2.4556563053489393

ts, yaws_mag, yaws_gyro, complems, kalmans = [], [], [], [], []

while(1):
    try:
        dt = time() - dt
        mb.time += dt
        dt = time()
        
        mb.telemetry()


        yaw_gyro, gyros = yaw_from_gyro_euler(gyros, mb)       
        yaw_mag, yaw_kf = kalman_filter_get_prediction(kf, mb, gyro_std=gyro_std, yaw_std=yaw_std)
        yaw_cf = complementary_filter(yaw_mag, yaw_gyro, alpha = 0.3)
        
        ts.append(round(mb.time, 3))
        yaws_mag.append(yaw_mag)
        yaws_gyro.append(mb.gyroz)
        complems.append(yaw_cf)
        kalmans.append(yaw_kf)

        sdata = "{:.3f} {} {} {} {} {} {} {}".format(mb.time, mb.gyroz, mb.magx, mb.magy, yaw_mag, yaw_gyro, yaw_cf, yaw_kf) 
#        print(sdata)
        try:
            if(mb.dist1 < obstacle_threshold and mb.dist2 < obstacle_threshold):
                rul = car.turn_center()
                motors = car.move_backward()
                print("Center obstacle")
#                sleep(0.1)

            if(mb.dist1 < obstacle_threshold):
                rul = car.turn_right()
                motors = car.move_backward()
                print("Obstacle from the left")
#                sleep(0.1)

            if(mb.dist2 < obstacle_threshold):
                rul = car.turn_left()
                motors = car.move_backward()
                print("Obstacle from the right")
#                sleep(0.1)
        
            if(mb.motors_voltage < voltage_threshold):
                uv_counter += 1
                if uv_counter > 4:
                    motors = car.stop()
                    rul = car.turn_center()
                    print("Undervoltage!!")
                    f.close()
                    break
        
            if(mb.dist1 >= obstacle_threshold and mb.dist2 >= obstacle_threshold and mb.motors_voltage >= voltage_threshold):
                motors = car.move_forward()
                rul = car.turn_right()
                uv_counter = 0
                print(sdata)
        
        
        except Exception:
            pass

        
        f.write(sdata)
        f.write('\n')

    except KeyboardInterrupt:
        car.stop()
        car.turn_center()
        f.close()
        legend = ['yaw from mag', 'yaw from gyro', 'complementary', 'kalman']
        plot_data_n_labels(ts, [yaws_mag, yaws_gyro, complems, kalmans], title = "Yaw", xlabel = 'Time, s',
                           ylabel = 'Yaw, degs', legend = legend)
        break