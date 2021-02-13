#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sat Dec 12 15:56:29 2020

@author: root
"""

from LIBRARY.rpi_car import rpi_movement
from LIBRARY.rpi_car_mag_calibration import write_calibration_file
from LIBRARY.rpi_telemetry import mb_telemetry
from LIBRARY.rpi_orientation import rpi_orientation

from time import time

from filterpy.kalman import UnscentedKalmanFilter
from filterpy.common import Q_discrete_white_noise
from filterpy.kalman import JulierSigmaPoints


import numpy as np

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
orientation = rpi_orientation()

yaw_gyro,yaw_mag,yaw_mag_compensated =0,0,0

ts, yaws_gyro, yaws_mag, yaws_mag_compensated = [], [], [], []
rolls, pitchs = [], []



sigmas = JulierSigmaPoints(n=2, kappa=1)
def fx(x, dt):
    xout = np.empty_like(x)
    xout[0] = x[1] * dt + x[0]
    xout[1] = x[1]
    return xout

def hx(x):
    return x[:1] # return position [x]

ukf = UnscentedKalmanFilter(dim_x=2, dim_z=1, dt=1., hx=hx, fx=fx, points=sigmas)
ukf.P *= 10
ukf.R *= .5
ukf.Q = Q_discrete_white_noise(2, dt=1., var=0.03)

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
        
        rolls.append(orientation.roll_from_acc(mb))
        pitchs.append(orientation.pitch_from_acc(mb))

        yaw_gyro = orientation.yaw_from_gyros(gyros, ts, gyros[0])
        yaw_mag = orientation.yaw_from_mags(mb.magx,mb.magy,mb.magz, norm = False)
        yaw_mag_compensated=orientation.yaw_from_mags_tilt_compensate(mb, norm = False)
      
        yaws_gyro.append(yaw_gyro)
        yaws_mag.append(yaw_mag)
        yaws_mag_compensated.append(yaw_mag_compensated)

        sdata = "{:.3f} {} {} {} {} {} {}".format(mb.time, mb.gyroz, mb.magx, mb.magy, yaw_mag, yaw_gyro, yaw_mag_compensated) 

        try:
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
        try:      

            legend = ['magx', 'magy', 'magz']
            orientation.plot_data_n_labels(ts, [magxs, magys, magzs], title = "IMU data", xlabel = 'Time, s',
                               ylabel = 'Magnetometer, degs', legend = legend)
        except Exception:
            pass
        break