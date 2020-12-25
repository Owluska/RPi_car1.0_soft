#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sat Dec 12 16:18:38 2020

@author: root
"""
import wiringpi
import FaBo9Axis_MPU9250
#!pip3 install pi-ina219
from ina219 import INA219
from time import time, sleep

class mb_telemetry():
    def __init__(self):
        self.imu = None
        
        self.ina = None
        self.SHUNT_OHMS = 0.01
        self.motors_voltage = None
        self.motors_current = None
        
        self.US1 = None
        self.US1_TRIG = 22
        self.US1_ECHO = 17
        
        self.US2 = None
        self.US2_TRIG = 24
        self.US2_ECHO = 23
        self.T = 25
        self.sound_vel=(331.5+0.6*self.T)
        self.us = 1e-6
        self.ms = 1e-3
        self.timeout = 10 * self.ms
        self.dist1 = None
        self.dist2 = None
        
        self.mpu = None
        self.accx = None
        self.accy = None
        self.gyroz = None
        self.magx = None
        self.magy = None
        self.magx_offset = -4.969
        self.magx_scale = 1.466
        self.magy_offset = 24.116
        self.magy_scale = 0.874
        self.magz_offset = 24.710
        self.magz_scale = 0.853
        
        self.time = 0

    def setup_mpu9250(self):
        try:
            self.imu=FaBo9Axis_MPU9250.MPU9250()
        except Exception:
            self.imu=None
        return self.imu    

    
    def setup_ina219(self):
        try:
            self.ina = INA219(self.SHUNT_OHMS)
            self.ina.configure()
        except Exception:
            self.ina = None
        return self.ina
    
    def setup_US_ports(self, triger, echo):
        wiringpi.pinMode(echo, wiringpi.GPIO.INPUT)
    
        wiringpi.pinMode(triger, wiringpi.GPIO.OUTPUT)
        wiringpi.digitalWrite(triger, wiringpi.GPIO.LOW)
    
    def init_all(self):
        self.ina = self.setup_ina219()
        self.imu = self.setup_mpu9250()
        self.US1 = self.setup_US_ports(self.US1_TRIG,self.US1_ECHO)
        self.US2 = self.setup_US_ports(self.US2_TRIG,self.US2_ECHO)
    
    def get_data_ina219(self):
        if self.ina != None:
            self.motors_voltage = round(self.ina.voltage(), 2)
            self.motors_current = round(self.ina.current(), 2)            
            return self.motors_voltage, self.motors_current
        else:
            return None, None

    def get_mpu9250_acc(self):
        if self.imu != None:
            acc = self.imu.readAccel()
            self.accx = acc['x']
            self.accy = acc['y']
            return self.accx, self.accy
        else:
            return None, None
        
    def get_mpu9250_gyro(self):
        if self.imu != None:
            gyro = self.imu.readGyro()
            self.gyroz = gyro['z']
            return self.gyroz
        else:
            return None
    
    def get_mpu9250_mag(self):
        if self.imu != None:
            mag = self.imu.readMagnet()
            self.magx = round((mag['x'] - self.magx_offset) * self.magx_scale, 2)
            self.magy = round((mag['y'] - self.magy_offset) * self.magy_scale, 2)
            return self.magx, self.magy
        else:
            return None, None    
    
    def get_distance(self, triger, echo):
#        sleep(0.05)
        pulse_start = 0
        pulse_end = 0
        
        wiringpi.digitalWrite(triger, wiringpi.GPIO.LOW)
        sleep(0.05)
        
        wiringpi.digitalWrite(triger, wiringpi.GPIO.HIGH)
        sleep(10*self.us)
        wiringpi.digitalWrite(triger, wiringpi.GPIO.LOW)
        
        start_time = time()
        while(wiringpi.digitalRead(echo) == wiringpi.GPIO.LOW):
            pulse_start = time()
            if(pulse_start - start_time > self.timeout):
                break
            
        start_time = time()
        while(wiringpi.digitalRead(echo) == wiringpi.GPIO.HIGH):
            pulse_end = time()
            if(pulse_end - start_time > self.timeout):
                break
    
     
        distance = self.sound_vel * (pulse_end - pulse_start) *0.5*100
        distance = abs(round(distance, 2))
        
           
        if distance < 0 or distance > self.sound_vel * (self.timeout) * 100:
                distance = None            
        return distance

    def telemetry(self):    
        self.time = round(self.time, 3)
        self.motors_voltage, self.motors_current = self.get_data_ina219()
        sleep(0.01)
        
        self.accx, self.accy = self.get_mpu9250_acc()
        sleep(0.01)
        
        self.gyroz = self.get_mpu9250_gyro()
        sleep(0.01)
        
        self.magx, self.magy = self.get_mpu9250_mag()
        sleep(0.01)
        
        self.dist1 = self.get_distance(self.US1_TRIG, self.US1_ECHO)
        sleep(0.038)
        self.dist2 = self.get_distance(self.US2_TRIG, self.US2_ECHO)
        sleep(0.038)
        
        return [self.time, self.motors_voltage, self.motors_current, self.accx, self.accy, 
                self.gyroz, self.magx, self.magy, self.dist1, self.dist2]
        
    def read_calibration_file(self, path = '/home/pi/Desktop/RPi_car1.0_soft/calibration_data/mag'):
        mf = open(path)
        calib_str = mf.read()

        self.magx_offset = float(calib_str[10:17])
        self.magx_scale = float(calib_str[27:34])
        self.magy_offset = float(calib_str[45:52])
        self.magy_scale = float(calib_str[62:69])