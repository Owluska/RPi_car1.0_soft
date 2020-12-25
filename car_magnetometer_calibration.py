#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sat Dec 12 15:56:29 2020

@author: root
"""

from LIBRARY.rpi_car import rpi_movement
from time import sleep, time
import FaBo9Axis_MPU9250

car = rpi_movement()
car.init()

imu=FaBo9Axis_MPU9250.MPU9250()


def eight_moving(car, mags):
    if car == None:
        print("Car data is empty, returning")
        return
     
    car.move_forward()
     
    car.turn_right()
    for i in range(50):
        mags.append(imu.readMagnet())
        sleep(0.01)
#    sleep(0.5)
     
    car.turn_left()
    for i in range(320):
        mags.append(imu.readMagnet())
        sleep(0.01)
#    sleep(3.5)     
    
    car.turn_center()
    for i in range(80):
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
    
def calibrate(car, imu, auto = False):
    mags = []
    if auto:
        print("Ensure that there is a place about 3 m x 2 m!")
        mags = eight_moving(car, mags)        
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
    
    avg_delta = (avg_delta_x + avg_delta_y +avg_delta_z) / 3
    
    scale_x = avg_delta / avg_delta_x
    scale_y = avg_delta / avg_delta_y
    scale_z = avg_delta / avg_delta_z
    
    result = "{:6.3f} {:6.3f} {:6.3f} {:6.3f} {:6.3f} {:6.3f}".format(offset_x, offset_y, offset_z, scale_x, scale_y, scale_z)
    print(result)
    
    return offset_x, offset_y, offset_z, scale_x, scale_y, scale_z

 
     
#t = time()    
#offset_x, offset_y, offset_z, scale_x, scale_y, scale_z = calibrate(car, imu, auto = True)
#t = time()-t
#result = "{:1.1f}:{:6.3f} {:6.3f} {:6.3f} {:6.3f} {:6.3f} {:6.3f}".format(t, offset_x, offset_y, offset_z, scale_x, scale_y, scale_z)
#print(result)