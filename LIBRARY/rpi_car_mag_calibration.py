#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sat Dec 12 15:56:29 2020

@author: root
"""

from LIBRARY.rpi_car import rpi_movement
from time import sleep
import FaBo9Axis_MPU9250
from datetime import datetime

car = rpi_movement()
car.init()




def eight_moving(imu, car, mags):
    if car == None:
        print("Car data is empty, returning")
        return
     
    car.move_forward()
     
    car.turn_right()
    for i in range(30):
        mags.append(imu.readMagnet())
        sleep(0.01)
#    sleep(0.5)
     
    car.turn_left()
    for i in range(320):
        mags.append(imu.readMagnet())
        sleep(0.01)
#    sleep(3.5)     
    
    car.turn_center()
    for i in range(50):
        mags.append(imu.readMagnet())
        sleep(0.01)
#    sleep(2)
     
    car.turn_right()
    for i in range(300):
        mags.append(imu.readMagnet())
        sleep(0.01)
#    sleep(3)
    
    car.turn_center()
    for i in range(20):
        mags.append(imu.readMagnet())
        sleep(0.01)
    car.stop()
    return mags
    
def mag_calibrate(car, auto = False):
    imu=FaBo9Axis_MPU9250.MPU9250()
    if imu == None:
        print("!")
        return
    mags = []
    if auto:
        print("Ensure that there is a place about 3 m x 2 m!")
        for i in range(500):
            if i%100 == 0:
                print("--->" * int(i/100), end ='')
            sleep(0.01)
        
        print()
        mags = eight_moving(imu, car, mags)

    else:
        print("Slowly move car in eight figure pattern")
        print("Measuring magnetic field....To stop press ""CTRL+C")
        while(1):
            try:
                mags.append(imu.readMagnet())
                sleep(0.1)
            except KeyboardInterrupt:
                print("Measuring stopped.")
                break

    xs = [m['x'] for m in mags]
    ys = [m['y'] for m in mags]
    zs = [m['z'] for m in mags]
    
    #hard iron coefficient calculation    
    offset_x = (max(xs) + min(xs))/2
    offset_y = (max(ys) + min(ys))/2
    offset_z = (max(zs) + min(zs))/2
    
    #sof iron coefficients calculation
    avg_delta_x = (max(xs) - min(xs))/2
    avg_delta_y = (max(ys) - min(ys))/2
    avg_delta_z = (max(zs) - min(zs))/2
    
    avg_delta = (avg_delta_x + avg_delta_y + avg_delta_z) / 3
    
    scale_x = avg_delta / avg_delta_x
    scale_y = avg_delta / avg_delta_y
    scale_z = avg_delta / avg_delta_z
    
#    result = "{:6.3f} {:6.3f} {:6.3f} {:6.3f} {:6.3f} {:6.3f}".format(offset_x, offset_y, offset_z, scale_x, scale_y, scale_z)
    result = "{:7.3f} {:7.3f} {:7.3f} {:7.3f} {:7.3f} {:7.3f}".format(offset_x, scale_x, offset_y, scale_y, offset_z, scale_z)
    print(result)
    
    return offset_x, offset_y, offset_z, scale_x, scale_y, scale_z

def create_file(name = 'test', path = '/home/pi/Desktop/RPi_car1.0_soft/output/', label = '', addData = False):
    if addData:
        now = datetime.now()
        date_time = now.strftime("_%m_%d_%Y_%H_%M_%S")
        name += date_time
    f = open(path+name, 'w')
    return f

def write_calibration_file(car, auto = False, attempts = 1, path = '/home/pi/Desktop/RPi_car1.0_soft/calibration_data/'):
    name = 'mag'
    mf = create_file(name = name,  path = path)
    data = []
    offset_x, scale_x, offset_y, scale_y, offset_z, scale_z = 0,0,0,0,0,0
    for i in range(attempts):
#        
#        offset_x += offset_x
#        scale_x += scale_x
#        offset_y += offset_y
#        scale_y += scale_y
        
        data.append(mag_calibrate(car, auto = auto))
        
    for d in data:
        offset_x += d[0]
        scale_x += d[3]
        offset_y += d[1]
        scale_y += d[4]
        offset_z += d[2]
        scale_z += d[5]
        
    offset_x /= attempts
    scale_x /= attempts
    offset_y /= attempts
    scale_y /= attempts
    offset_z /= attempts
    scale_z /= attempts
    label = "x_offset x_scale y_offset y_scale z_offset z_scale\n"
    mf.write(label)    
    
    s = "{:.3f} {:.3f} {:.3f} {:.3f} {:.3f} {:.3f}\n".format(offset_x,
                   scale_x, offset_y, scale_y, offset_z, scale_z)
    mf.write(s)
    mf.close()
    
     
#t = time()    
#offset_x, offset_y, offset_z, scale_x, scale_y, scale_z = calibrate(car, imu, auto = True)
#t = time()-t
#result = "{:1.1f}:{:6.3f} {:6.3f} {:6.3f} {:6.3f} {:6.3f} {:6.3f}".format(t, offset_x, offset_y, offset_z, scale_x, scale_y, scale_z)
#print(result)
