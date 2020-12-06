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
import os



RUL = 12
MOTOR_A = 6  
MOTOR_B = 26

RUL_CENTER = 150
RUL_RIGHT = 180
RUL_LEFT = 120

SHUNT_OHMS = 0.01

US1_TRIG = 22
US1_ECHO = 17

US2_TRIG = 24
US2_ECHO = 23

us = 1e-6
ms = 1e-3

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
    wiringpi.pwmWrite(RUL, RUL_CENTER)
    wiringpi.digitalWrite(MOTOR_A, wiringpi.GPIO.HIGH)
    wiringpi.digitalWrite(MOTOR_B, wiringpi.GPIO.HIGH)
    return 'S', 'C'
    
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
        pulse_start = time() - start_time
        if(pulse_start > timeout):
            if toPrint == True:
                print("Can't measure pulse, please insure the sensor is connected!")
            break
        
    start_time = time()
    while(wiringpi.digitalRead(echo) == wiringpi.GPIO.HIGH):
        pulse_end = time() - start_time
        if(pulse_end > timeout):
            if toPrint == True:
                print("Can't measure pulse, please insure the sensor is connected!")
            break

 
    distance = sound_vel* (pulse_end - pulse_start)*0.5*100
    distance = abs(round(distance, 2))
    
    if toPrint == True:   
        if distance > 0 and distance < 400:
            print("Measured distance, cm: {}".format(distance))
        else:
            print("No obstacle")
       
    return distance

def telemetry(velocity, rotation, ina, imu):
    voltage, current = get_data_ina219(ina)
    if voltage < 7.0:
        print("Undervoltage detected, shutdown in 1 s")
        sleep(1)
        os.system('shutdown now -s')
    acc, gyro, mag = get_data_imu9250(imu)

    yaw = yaw_from_mag(mag)
    
    US1 = get_distance(US1_TRIG, US1_ECHO)
    US2 = get_distance(US2_TRIG, US2_ECHO)

    label = "Motors Rul Volts,V  Curs,mA  accX accY accZ gyroX gyroY gyroZ yaw,deg US1,cm US2,cm"
    print(label)

    string = "{:>6} {:>3} {:7.2f} {:8.2f} {:4.2f} {:4.2f} {:4.2f} {:5.2f} {:5.2f} {:5.2f} {:7.2f} {:6.2f} {:6.2f}\n".format(velocity, rotation, voltage,
              current, acc['x'], acc['y'], acc['z'], gyro['x'], gyro['y'], gyro['z'], yaw, US1, US2)
    print(string)

setup()
imu = setup_imu() 
ina = setup_ina219()  


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
while(1):
    try:
        i += 1
        if i%5 == 0:   
            velocity = move_forward()
            rotation = turn_left()
        elif i%5 == 1:
            velocity = move_forward()
            rotation = turn_right()
        elif i%5 == 2:
            velocity = move_backward()
            rotation = turn_right()
        elif i%5 == 3:
            velocity = move_backward()
            rotation = turn_left()
        else:
            velocity, rotation = stop()
            wiringpi.digitalWrite(MOTOR_A, wiringpi.GPIO.LOW)
            wiringpi.digitalWrite(MOTOR_B, wiringpi.GPIO.LOW)
    
        telemetry(velocity, rotation, ina, imu)
        sleep(.5)
    except KeyboardInterrupt:
        velocity, rotation = stop()
        wiringpi.digitalWrite(MOTOR_A, wiringpi.GPIO.LOW)
        wiringpi.digitalWrite(MOTOR_B, wiringpi.GPIO.LOW)
        break
    
