#!/usr/bin/env python3 
# -*- coding: utf-8 -*-
"""
Created on Mon Nov 30 15:38:35 2020

@author: root
"""

import wiringpi
from time import time, sleep
import math
import FaBo9Axis_MPU9250
#!pip3 install pi-ina219
from ina219 import INA219
#from ina219 import DeviceRangeError
from datetime import datetime



RUL = 12
MOTOR_A = 6  
MOTOR_B = 26

RUL_CENTER = 150
RUL_RIGHT = 200
RUL_LEFT = 100

SHUNT_OHMS = 0.01

US1_TRIG = 22
US1_ECHO = 17

US2_TRIG = 24
US2_ECHO = 23

us = 1e-6
ms = 1e-3

T = 25
sound_vel=(331.5+0.6*T)


def setup():
    wiringpi.wiringPiSetupGpio()
    
    wiringpi.pinMode(MOTOR_A, wiringpi.GPIO.OUTPUT)
    wiringpi.pinMode(MOTOR_B, wiringpi.GPIO.OUTPUT)

    wiringpi.pullUpDnControl(MOTOR_A, wiringpi.GPIO.PUD_UP)
    wiringpi.pullUpDnControl(MOTOR_B, wiringpi.GPIO.PUD_UP)
#    wiringpi.digitalWrite(MOTOR_A, wiringpi.GPIO.HIGH)
#    wiringpi.digitalWrite(MOTOR_B, wiringpi.GPIO.HIGH)
    
    wiringpi.pinMode(RUL, wiringpi.GPIO.PWM_OUTPUT)
    
    wiringpi.pwmSetMode(wiringpi.GPIO.PWM_MODE_MS)
    wiringpi.pwmSetClock(192)
    wiringpi.pwmSetRange(2000)
    

def stop():
    wiringpi.digitalWrite(MOTOR_A, wiringpi.GPIO.HIGH)
    wiringpi.digitalWrite(MOTOR_B, wiringpi.GPIO.HIGH)
    return 'S'
    
def move_forward():
    wiringpi.digitalWrite(MOTOR_A, wiringpi.GPIO.LOW)
    wiringpi.digitalWrite(MOTOR_B, wiringpi.GPIO.HIGH)
    return 'F'
    
def move_backward():
    wiringpi.digitalWrite(MOTOR_A, wiringpi.GPIO.HIGH)
    wiringpi.digitalWrite(MOTOR_B, wiringpi.GPIO.LOW)
    return 'B'
    
def turn_right():
    wiringpi.pwmWrite(RUL, RUL_RIGHT)
    return 'R'
    
def turn_left():
    wiringpi.pwmWrite(RUL, RUL_LEFT)
    return 'L'

def turn_center():
    wiringpi.pwmWrite(RUL, RUL_CENTER)
    return 'C'

def setup_imu():
    try:
        mpu=FaBo9Axis_MPU9250.MPU9250()
    except Exception:
        mpu=None
    return mpu    

    
def setup_ina219():
    try:
        ina = INA219(SHUNT_OHMS)
        ina.configure()
    except Exception:
        ina = None
    return ina

def get_data_ina219(ina):
    if ina != None:
        return ina.voltage(), ina.current()
    else:
        return None
    
def yaw_from_mag(data):
    r2d = 180 / math.pi
    my = data['y']
    mx = data['x']
    return math.atan2(-my, mx) * r2d

def get_data_imu9250(imu):
    if imu != None:
        return imu.readAccel(), imu.readGyro(), imu.readMagnet()
    else:
        return None
def setup_US_ports(triger, echo):
    wiringpi.pinMode(echo, wiringpi.GPIO.INPUT)

    wiringpi.pinMode(triger, wiringpi.GPIO.OUTPUT)
    wiringpi.digitalWrite(triger, wiringpi.GPIO.LOW)
    
def get_distance(triger, echo, timeout = 10 * ms, toPrint = False):
    pulse_start = 0
    pulse_end = 0
    
    wiringpi.digitalWrite(triger, wiringpi.GPIO.LOW)
    sleep(0.05)
    
    wiringpi.digitalWrite(triger, wiringpi.GPIO.HIGH)
    sleep(10*us)
    wiringpi.digitalWrite(triger, wiringpi.GPIO.LOW)
    
    start_time = time()
    while(wiringpi.digitalRead(echo) == wiringpi.GPIO.LOW):
        pulse_start = time()
        if(pulse_start - start_time > timeout):
            if toPrint == True:
                print("Can't measure pulse, please insure the sensor is connected!")
            break
        
    start_time = time()
    while(wiringpi.digitalRead(echo) == wiringpi.GPIO.HIGH):
        pulse_end = time()
        if(pulse_end - start_time > timeout):
            if toPrint == True:
                print("Can't measure pulse, please insure the sensor is connected!")
            break

 
    distance = sound_vel* (pulse_end - pulse_start)*0.5*100
    distance = abs(round(distance, 2))
    
       
    if distance > 0 and distance < sound_vel* (timeout)*0.5*100:
            if toPrint == True:
                print("Measured distance, cm: {}".format(distance))
    else:
            distance = None
            if toPrint == True:
                print("No obstacle")
       
    return distance

def telemetry(velocity, rotation, ina, imu, file, status = None):
    voltage, current = get_data_ina219(ina)
    #TODO handle pulse undervoltage
    if voltage < 7.0:
        status = "undervoltage"
    else:
        status = None
        
    acc, gyro, mag = get_data_imu9250(imu)

    yaw = yaw_from_mag(mag)
    
    US1 = get_distance(US1_TRIG, US1_ECHO)
    US2 = get_distance(US2_TRIG, US2_ECHO)
    #TODO handle None type and second US
    try:
        if US1 <= 10:
            status = "obstacle"
        else:
            status = None
    except:
        pass
    label = "Motors Rul Volts,V  Curs,mA  accX accY accZ gyroX gyroY gyroZ yaw,deg US1,cm US2,cm"
    print(label)
    
    string = "{:>6} {:>3} {:7.2f} {:8.2f} {:4.2f} {:4.2f} {:4.2f} {:5.2f} {:5.2f} {:5.2f} {:7.2f} {} {}\n".format(velocity, rotation, voltage,
              current, acc['x'], acc['y'], acc['z'], gyro['x'], gyro['y'], gyro['z'], yaw, US1, US2)
    file.write(string)
    print(string)
    return status

setup()
imu = setup_imu() 
ina = setup_ina219()  
setup_US_ports(US2_TRIG, US2_ECHO)
setup_US_ports(US1_TRIG, US1_ECHO)

name = 'test'
now = datetime.now()
date_time = now.strftime("_%m_%d_%Y_%H_%M_%S")
name += date_time
path = '/home/pi/Desktop/RPi_car1.0_soft/output/'
f = open(path+name, 'a+')
f.write('Motors Rul Volts,V  Curs,mA  accX accY accZ gyroX gyroY gyroZ yaw,deg US1,cm US2,cm\n')

sleep(2)

#print("start")
#move_forward()
#sleep(2)
#stop()
#   
#move_backward()
#sleep(2)
#stop()
#
#
#move_backward()
#turn_right()
#sleep(1)
#stop()
#
#
#
#move_backward()
#turn_left()
#sleep(1)
#stop()
#print("stop")
wiringpi.digitalWrite(MOTOR_A, wiringpi.GPIO.LOW)
wiringpi.digitalWrite(MOTOR_B, wiringpi.GPIO.LOW)

velocity = 'S'
rotation = 'C'

i = 0
status = None
while(1):
    try:
#        i += 1
#        if i%5 == 0:   
#            velocity = move_forward()
#            rotation = turn_left()
#        elif i%5 == 1:
        
#            velocity = move_forward()
#            rotation = turn_right()
#        elif i%5 == 2:
#            velocity = move_backward()
#            rotation = turn_right()
#        elif i%5 == 3:
#            velocity = move_backward()
#            rotation = turn_left()
#        else:
#            velocity, rotation = stop()
#            sleep(2)
#            wiringpi.digitalWrite(MOTOR_A, wiringpi.GPIO.LOW)
#            wiringpi.digitalWrite(MOTOR_B, wiringpi.GPIO.LOW)
        
        

    
        status = telemetry(velocity, rotation, ina, imu, f, status = status)
        if status == "obstacle":
            velocity = move_backward()
            rotation = turn_left()
            sleep(0.1)
            continue
        
        velocity = move_forward()
        rotation = turn_right()
        sleep(.1)
    except KeyboardInterrupt:
        velocity = stop()
        rotation = turn_center()
        f.close()
        break
    
