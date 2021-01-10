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
import matplotlib.pyplot as plt
from scipy import integrate


def normalize_3axis(x, y, z):
    n = math.sqrt(x**2 + y**2 + z**2)
    x /= n
    y /= n
    z /= n
    return x, y, z

def yaw_from_mags(mx, my, mz, norm = True):
    
    if norm:
        mx, my, mz = normalize_3axis(mx, my, mz)
    
    # if mx < 0:
    #     yaw = 180 - math.atan(my/mx) * 180/math.pi
    # elif mx > 0 and my < 0:
    #     yaw = - math.atan(my/mx) * 180/math.pi
    # elif mx > 0 and my > 0:
    #     yaw = 360 - math.atan(my/mx) * 180/math.pi
    # elif mx == 0 and my < 0:
    #     yaw = 90
    # elif mx == 0 and my > 0:
    #     yaw = 270
    # else:
    #     yaw = 0
    yaw = math.atan2(my, mx) * 180/math.pi
    
    yaw = round(yaw, 3)
    return yaw


def roll_from_acc(board):
    ay = board.accy
    az = board.accz - 1
    
    roll = math.atan2(-ay, az)
#    print("roll:", roll)
    return roll

def pitch_from_acc(board):
    ax = board.accx
    ay = board.accy
    az = board.accz - 1
    
    pitch = math.atan2(-ax, math.sqrt(ay**2 + az**2))
#    print("pitch:", pitch)
    return pitch
    
def yaw_from_mags_tilt_compensate(board, norm = True):
    
    roll = roll_from_acc(board)
    pitch = pitch_from_acc(board)
    
    mx = board.magx
    my = board.magy
    mz = board.magz
    
    # if norm:
    #     mx, my, mz = normalize_3axis(mx, my, mz)
    
    My = mx*math.cos(roll) + mz*math.sin(roll)
    Mx = mx*math.cos(pitch)  + my*math.sin(roll)*math.sin(pitch) - mz*math.sin(pitch)*math.cos(roll)
 
    yaw = yaw_from_mags(Mx, My, mz, norm = norm)
    return yaw
                    
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


dt = time()

t, gyro, magx, magy, magz = 0.15,0,0,0,0

magxs, magys, magzs, gyros = [], [], [], []
sdata = ''

mb.time = t
telemetry = mb.telemetry()



obstacle_threshold = 15
voltage_threshold = 6.7
uv_counter = 0


yaw_gyro,yaw_mag,yaw_mag_compensated =0,0,0

ts, yaws_gyro, yaws_mag, yaws_mag_compensated = [], [], [], []
rolls, pitchs = [], []
while(1):
    try:
        dt = time() - dt
        mb.time += dt
        dt = time()
        

        
        mb.telemetry()

        ts.append(round(mb.time, 3))
        gyros.append(mb.gyroz)
        magxs.append(mb.magx)
        magys.append(mb.magy)
        magzs.append(mb.magz)
        
        rolls.append(roll_from_acc(mb))
        pitchs.append(pitch_from_acc(mb))

#        yaw_gyro = yaw_from_gyro_euler(yaw_gyro, mb)
        yaw_gyro = yaw_from_gyros(gyros, ts, gyros[0])
        yaw_mag = yaw_from_mags(mb.magx,mb.magy,mb.magz, norm = False)
        yaw_mag_compensated=yaw_from_mags_tilt_compensate(mb, norm = False)
 
        
        yaws_gyro.append(yaw_gyro)
        yaws_mag.append(yaw_mag)
        yaws_mag_compensated.append(yaw_mag_compensated)

        sdata = "{:.3f} {} {} {} {} {} {}".format(mb.time, mb.gyroz, mb.magx, mb.magy, yaw_mag, yaw_gyro, yaw_mag_compensated) 
#        print(mb.dist1, mb.dist2)
        try:
#            if(mb.dist1 < obstacle_threshold and mb.dist2 < obstacle_threshold):
#                rul = car.turn_center()
#                motors = car.move_backward()
#                print("Center obstacle")
##                sleep(0.1)
#
#            if(mb.dist1 < obstacle_threshold):
#                rul = car.turn_right()
#                motors = car.move_backward()
#                print("Obstacle from the left")
##                sleep(0.1)
#
#            if(mb.dist2 < obstacle_threshold):
#                rul = car.turn_left()
#                motors = car.move_backward()
#                print("Obstacle from the right")
##                sleep(0.1)
#        
            if(mb.motors_voltage < voltage_threshold):
                uv_counter += 1
                if uv_counter > 4:
                    motors = car.stop()
                    rul = car.turn_center()
                    print("Undervoltage!!")
                    break
#        
#            if(mb.dist1 >= obstacle_threshold and mb.dist2 >= obstacle_threshold and mb.motors_voltage >= voltage_threshold):
            else:
                motors = car.move_forward()
                rul = car.turn_right()
                uv_counter = 0
                print(sdata)
        
        
        except Exception:
            pass

        


    except KeyboardInterrupt:
        car.stop()
        car.turn_center()

#        mb.stop()
        try:      
#            legend = ['roll', 'pitch']
#            plot_data_n_labels(ts, [rolls, pitchs], title = "Eulers first pair", xlabel = 'Time, s',
#                               ylabel = 'Yaw, degs', legend = legend)
            legend = ['yaw from gyro', 'yaw from mag', 'compesated']
            plot_data_n_labels(ts, [yaws_gyro, yaws_mag, yaws_mag_compensated], title = "Yaw", xlabel = 'Time, s',
                               ylabel = 'Yaw, degs', legend = legend)
            legend = ['magx', 'magy', 'magz']
            plot_data_n_labels(ts, [magxs, magys, magzs], title = "IMU data", xlabel = 'Time, s',
                               ylabel = 'Magnetometer, degs', legend = legend)
        except Exception:
            pass
        break