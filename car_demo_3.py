#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sat Dec 12 15:56:29 2020

@author: root
"""

from LIBRARY.rpi_car import rpi_movement
from LIBRARY.rpi_car_mag_calibration import write_calibration_file
from LIBRARY.rpi_telemetry import mb_telemetry
from time import sleep, time
from datetime import datetime

def create_file(name = 'test', path = '/home/pi/Desktop/RPi_car1.0_soft/output/', label = ''):
    now = datetime.now()
    date_time = now.strftime("_%m_%d_%Y_%H_%M_%S")
    name += date_time
    f = open(path+name, 'a+')
    f.write(label)
    f.write('\n')
    return f

car = rpi_movement()
car.init()
mb = mb_telemetry()
mb.init_all()

toCalibrate = False
attempts = 5
if toCalibrate:
  write_calibration_file(car, auto = False, attempts = attempts)
  mb.read_calibration_file()
  print(mb.magx_offset, mb.magx_scale, mb.magy_offset, mb.magy_scale)
    

label = "Motors Rul time,s Volts,V  Curs,mA  accX accY gyroZ magX magY US1,cm US2,cm"
f = create_file(label = label)


dt = time()
rul = 'C'
motors = 'S'

#t = time()

while(1):
    try:
        dt = time() - dt
        mb.time += dt
        print(mb.telemetry())
        dt = time()
#        print("dt: ", t)
        
        
        if(mb.dist1 != None and mb.dist2 != None):
            if(mb.dist1 < 20 and mb.dist2 < 20):
                rul = car.turn_center()
                motors = car.move_backward()
#                sleep(0.1)
        if(mb.dist1 != None):
            if(mb.dist1 < 20):
                rul = car.turn_right()
                motors = car.move_backward()
#                sleep(0.1)
        if(mb.dist2 != None):        
            if(mb.dist2 < 20):
                rul = car.turn_left()
                motors = car.move_backward()
#                sleep(0.1)
        if(mb.dist1 != None and mb.dist2 != None): 
            if(mb.dist1 > 10 and mb.dist2 > 10):
                motors = car.move_forward()
                rul = car.turn_right()
        if(mb.motors_voltage != None):       
            if(mb.motors_voltage < 6.4):
                motors = car.stop()
                rul = car.turn_center()
                print("Undervoltage!!")
                f.close()
                break

        f.write(motors + ' ' + rul + ' ')
        for d in mb.telemetry():
            f.write(str(d)+' ')
        f.write('\n')

    except KeyboardInterrupt:
        car.stop()
        car.turn_center()
        f.close()
        break