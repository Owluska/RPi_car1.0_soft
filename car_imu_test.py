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

def create_file(name = 'test', path = '/home/pi/Desktop/RPi_car1.0_soft/output/', label = ''):
    now = datetime.now()
    date_time = now.strftime("_%m_%d_%Y_%H_%M_%S")
    name += date_time
    f = open(path+name, 'a+')
    f.write(label)
    f.write('\n')
    return f

def yaw_from_mags(telemetry):
    magx = telemetry[6]
    magy = telemetry[7]
    R2D = 180/math.pi
    yaw = round(math.atan2(-magy, magx) * R2D, 3)
    return magx, magy, yaw

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
    

def yaw_from_gyro_euler(gyros, telemetry):
    dt = telemetry[0]
    gyros.append(telemetry[5])
    length = len(gyros)
    if length == 0:
        return 0
    elif length == 1:
        yaw = gyros[0] * dt
    else:
        yaw = gyros[-2] + gyros[-1]*dt

    if abs(yaw) > 180:
        yaw = 180 + yaw % (-360)
    return dt, gyros[-1], round(yaw, 3)


def complementary_filter(data1, data2, alpha = 1):
    output = data1 * alpha + data2 * (1 - alpha)
    output = round(output, 3)
    return output

def kalman_filter_init(telemetry, yaw = 0, gyro = 0):
    tracker = KalmanFilter(dim_x=2, dim_z=2)
    dt = telemetry[0]
    tracker.F = np.array([[1, dt],
                          [0,  1]])
    tracker.H = np.array([[1, 0],
                          [0, 1]])
    
    gyroz_std = 0.04659871242856395
    yaw_rm_std = 2.4556563053489393
    tracker.R = np.array([[gyroz_std,          0],
                          [        0, yaw_rm_std]])
    
    Q = Q_discrete_white_noise(dim=2, dt = dt, var = 0.1)
#    Q = np.array([[Q[0][0], Q[0][1]],
#                  [Q[1][0], Q[1][1]]])
    tracker.Q = Q
    tracker.P = np.eye(2) * 10 # +/- 10 deg
    
    z_0 = [yaw, gyro]
    tracker.z = np.array(z_0)
    tracker.x = np.array(z_0)
    return tracker
car = rpi_movement()
car.init()
mb = mb_telemetry()
mb.init_all()


toCalibrate = False
attempts = 5
if toCalibrate:
  write_calibration_file(car, auto = False, attempts = attempts)
  print(mb.magx_offset, mb.magx_scale, mb.magy_offset, mb.magy_scale)

mb.read_mag_calibration_file()    

label = "time gyroz magX magY yaw_rm yaw_rg yaw_cf yaw_kf"
f = create_file(name = 'filters', label = label)

dt = time()

t, gyro, magx, magy, yaw_rm, yaw_rg, yaw_cf, yaw_kf = 0.15,0,0,0,0,0,0,0
gyros = []
sdata = ''

mb.time = t
telemetry = mb.telemetry()
magx, magy, yaw_rm = yaw_from_mags(telemetry)
kf = kalman_filter_init(telemetry, yaw = yaw_rm, gyro = telemetry[5])
#print(kf)

obstacle_threshold = 15
voltage_threshold = 6.7
uv_counter = 0
while(1):
    try:
        dt = time() - dt
        mb.time += dt
        telemetry = mb.telemetry()
        dt = time()
        t, gyro, yaw_rg = yaw_from_gyro_euler(gyros, telemetry)
        magx, magy, yaw_rm = yaw_from_mags(telemetry)
        yaw_cf = complementary_filter(yaw_rm, yaw_rg, alpha = 0.75)
        
        kf.z = np.array([yaw_rm, telemetry[5]])
        kf.predict()
        kf.update(kf.z)
        yaw_kf = round(kf.x[0], 3)
        
        sdata = "{} {} {} {} {} {} {} {}".format(t, gyro, magx, magy, yaw_rm, yaw_rg, yaw_cf, yaw_kf) 
        
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
        break