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
#import threading
#from multiprocessing.pool import ThreadPool

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
        self.g = 9.780318
        self.accx = None
        self.accy = None
        self.accz = None
        
        self.gyrox = None
        self.gyroy = None
        self.gyroz = None
        
        self.magx = None
        self.magy = None
        self.magx_offset = 0
        self.magx_scale = 1
        self.magy_offset = 0
        self.magy_scale = 1
        self.magz_offset = 0
        self.magz_scale = 1
        
        self.time = 0
        
#        self.setup_US_ports(self.US1_TRIG,self.US1_ECHO)
#        self.setup_US_ports(self.US2_TRIG,self.US2_ECHO)
#        
#        self.pool = ThreadPool(processes = 3)
#        self.result = self.pool.apply_async(self.get_distance_loop, (self,))
        

        
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
#            return self.motors_voltage, self.motors_current
#        else:
#            return None, None

    def get_mpu9250_acc(self):
        if self.imu != None:
            acc = self.imu.readAccel()
            #self.g = 1
            self.accx = acc['x'] * self.g
            self.accy = acc['y'] * self.g
            self.accz = acc['z'] * self.g
#            return self.accx, self.accy, self.accz
#        else:
#            return None, None, None
        
    def get_mpu9250_gyro(self):
        if self.imu != None:
            gyro = self.imu.readGyro()
            self.gyrox = gyro['x']
            self.gyroy = gyro['y']
            self.gyroz = gyro['z']
#            return self.gyrox, self.gyroy, self.gyroz
#        else:
#            return None, None, None
    
    def get_mpu9250_mag(self):
        if self.imu != None:
            mag = self.imu.readMagnet()
            self.magx = round((mag['x'] - self.magx_offset) * self.magx_scale, 3)
            self.magy = round((mag['y'] - self.magy_offset) * self.magy_scale, 3)
            self.magz = round((mag['z'] - self.magz_offset) * self.magz_scale, 3)
#            return self.magx, self.magy, self.magz
#        else:
#            return None, None, None
    
    
    def get_distance(self, US):
#        sleep(0.38)
#        sleep(0.05)
        pulse_start = 0
        pulse_end = 0
        if US == 1:
            triger = self.US1_TRIG
            echo = self.US1_ECHO            
        elif US == 2:
            triger = self.US2_TRIG
            echo = self.US2_ECHO
        
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
    
#    def get_distance_loop(self):
#        while(1):
#            self.dist1 = self.pool.apply_async(self.get_distance, (1)).get(timeout = 1)        
#            self.dist2 = self.pool.apply_async(self.get_distance, (2)).get(timeout = 1)
#            print(self.dist1, self.dist2)
            

    def telemetry(self):    
#        self.dist1 = self.pool.apply_async(self.get_distance, (1,)).get(timeout = .5)  
        self.time = round(self.time, 3)
        self.dist1 = self.get_distance(1) 
        
        self.get_data_ina219()
        sleep(0.020)
        
        self.get_mpu9250_acc()
#        sleep(0.010)

        self.dist2 = self.get_distance(2)        
        self.get_mpu9250_gyro()
#        sleep(0.010)
        
        self.get_mpu9250_mag()
        sleep(0.020)
   

        


#      
#        self.dist2 = self.pool.apply_async(self.get_distance, (2,)).get(timeout = .5)

        


        
    def read_mag_calibration_file(self, path = '/home/pi/Desktop/RPi_car1.0_soft/calibration_data/mag'):
        mf = open(path)
        calib_str = mf.read()
        data = []
        temp = ''
        start = calib_str.find('\n') + 1
        calib_str = calib_str[start:]
        for c in calib_str:
            if c ==' ' or c == '\n':
                data.append(temp)
                temp = ''
            temp += c
#        print(data)     
        self.magx_offset = float(data[0])
        self.magx_scale = float(data[1])
        self.magy_offset = float(data[2])
        self.magy_scale = float(data[3])
        self.magz_offset = float(data[4])
        self.magz_scale = float(data[5])