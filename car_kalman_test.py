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
from scipy import integrate

def create_file(name = 'test', path = '/home/pi/Desktop/RPi_car1.0_soft/output/', label = ''):
    now = datetime.now()
    date_time = now.strftime("_%m_%d_%Y_%H_%M_%S")
    name += date_time
    f = open(path+name, 'a+')
    f.write(label)
    f.write('\n')
    return f

def yaw_from_mags(board, norm = True):
    magx = board.magx
    magy = board.magy
    magz = board.magz
    
    if norm:
        n = math.sqrt(magz**2 + magy**2 + magx**2)
        magy /= n
        magx /= n
    
    yaw = math.atan(-magy/magx) * 180/math.pi
    yaw = round(yaw, 3)
    return yaw

def yaw_from_mags_tilt_compensate(board, norm = True):
    ax = board.accx
    ay = board.accy
    az = board.accz
    
    roll = math.atan(ay/az)
    pitch = math.atan(-ax/math.sqrt(ay**2 + az**2))
    
    magx = board.magx
    magy = board.magy
    magz = board.magz
    
    if norm:
        n = math.sqrt(magx**magx + magy**magy + magz**magz)
        magx /= n
        magy /= n
        magz /= n
    
    Mx = magx*math.cos(pitch) + magz*math.sin(pitch)
    My = magx*math.sin(roll)*math.sin(pitch) + magy*math.cos(roll) - magz*math.sin(roll)*math.cos(pitch)

    yaw = math.atan(My/Mx)
    return round(yaw, 3)                    
def yaw_from_gyro_euler(previous_yaw, board):
    dt = board.time  

    yaw = previous_yaw + board.gyroz*dt

    if abs(yaw) > 180:
        yaw = -180 + yaw % (360)
    yaw = round(yaw,3)
    return yaw

def yaw_from_gyros(gyros, times, init):

    yaw = integrate.cumtrapz(gyros, times, initial = init)[-1]

    if abs(yaw) > 180:
        yaw = -180 + yaw % (360)
    yaw = round(yaw,3)
    return yaw



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
    plt.grid()
    
    if legend != None:
        plt.legend(legend)


car = rpi_movement()
car.init()
mb = mb_telemetry()
mb.init_all()


toCalibrate = False 
attempts = 3
if toCalibrate:
  write_calibration_file(car, auto = False, attempts = attempts)
  

mb.read_mag_calibration_file()
print("Magnetometer caibration:")    
print(mb.magx_offset, mb.magx_scale, mb.magy_offset, mb.magy_scale, mb.magz_offset, mb.magz_scale)

label = "time gyroz magX magY yaw_mag yaw_gyro yaw_cf yaw_kf"
print(label)
f = create_file(name = 'filters', label = label)

dt = time()

t, gyro, magx, magy, magz = 0.15,0,0,0,0
yaw_rm, yaw_rg, yaw_kf = 0,0,0
magxs, magys, magzs, gyros = [], [], [], []
sdata = ''

mb.time = t
telemetry = mb.telemetry()
kf = kalman_filter_init(mb, yaw = yaw_from_mags(mb), gyro = mb.gyroz)
yaw_gyro = yaw_from_gyro_euler(yaw_from_mags(mb), mb) 
#print(kf)

obstacle_threshold = 15
voltage_threshold = 6.7
uv_counter = 0

gyro_std, yaw_std = 0.04659871242856395,2.4556563053489393


ts, yaws_mag, yaws_gyro, kalmans = [], [], [], []

while(1):
    try:
        dt = time() - dt
        mb.time += dt
        dt = time()
        
#        
#        yaws, gyros, magxs, magys = [], [], [], []
#        for i in range(3):
#            mb.telemetry()
#            gyros.append(mb.gyroz)
#            magxs.append(mb.magx)
#            magys.append(mb.magy)
#            yaws.append(yaw_from_mags(mb))
#       
#        gyro_std = np.array(gyros).std()
#        yaw_std = np.array(yaws).std()
#        
#        print(yaw_std, gyro_std)
#        mb.magx = np.median(magxs)
#        mb.magy = np.median(magys)
        
        mb.telemetry()
        
        ts.append(round(mb.time, 3))
        gyros.append(mb.gyroz)
        magxs.append(mb.magx)
        magys.append(mb.magy)
        magzs.append(mb.magz)

#        yaw_gyro = yaw_from_gyro_euler(yaw_gyro, mb)
        yaw_gyro = yaw_from_gyros(gyros, ts, gyros[0])
        yaw_mag, yaw_kf = kalman_filter_get_prediction(kf, mb, gyro_std=gyro_std, yaw_std=yaw_std)

        

        yaws_mag.append(yaw_mag)
        yaws_gyro.append(yaw_gyro)
        kalmans.append(yaw_kf)

        sdata = "{:.3f} {} {} {} {} {} {}".format(mb.time, mb.gyroz, mb.magx, mb.magy, yaw_mag, yaw_gyro, yaw_kf) 
#        print(mb.dist1, mb.dist2)
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
#            else:
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
#        mb.stop()
        try:
            legend = ['yaw from mag', 'yaw from gyro', 'kalman']
            plot_data_n_labels(ts, [yaws_mag, yaws_gyro, kalmans], title = "Yaw", xlabel = 'Time, s',
                               ylabel = 'Yaw, degs', legend = legend)
            legend = ['magx', 'magy', 'magz']
            plot_data_n_labels(ts, [magxs, magys, magzs], title = "IMU data", xlabel = 'Time, s',
                               ylabel = 'Magnetometer, degs', legend = legend)
        except Exception:
            pass
        break