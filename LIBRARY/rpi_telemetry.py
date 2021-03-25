#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sat Dec 12 16:18:38 2020

@author: root
"""
from ina219 import INA219
from mpu9250_i2c import mpu6050_conv, AK8963_conv
import numpy as np
import csv
from time import sleep

class mb_telemetry():
    def __init__(self):
        self.imu = None
        
        self.ina1 = None
        self.ina2 = None
        
        self.mb_adress = 0x40
        self.dc_adress = 0x41
        
        self.SHUNT_OHMS = 0.01
        
        self.motors_voltage = None
        self.motors_current = None
        
        self.rpi_voltage = None
        self.rpi_current = None
        
        self.g = 9.84
        
        self.accx = None
        self.accy = None
        self.accz = None
        
        
        self.D2R = 3.14/180
        self.gyrox = None
        self.gyroy = None
        self.gyroz = None
        
        self.magx = None
        self.magy = None
        self.magz = None

        
        self.time = 0
        
        self.file = 'calibration_data/mpu9250_cal_params.csv'       
        self.offsets = self.read_calibration_file()

        

    
    def setup_ina219(self, adress):
        try:
            ina = INA219(self.SHUNT_OHMS, address=adress)
            ina.configure()
        except Exception:
            ina = None
        return ina
    
    
    def init_all(self):
        self.ina1 = self.setup_ina219(self.mb_adress)
        self.ina2 = self.setup_ina219(self.dc_adress)

    
    def get_data_ina219(self, ina):
        if ina != None:
            voltage = round(ina.voltage(), 2)
            current = round(ina.current(), 2)            
            return voltage, current
        else:
            return None, None

    def get_mpu9250_data(self):
            self.accx,self.accy,self.accz,self.gyrox,self.gyroy,self.gyroz = mpu6050_conv() # read and convert mpu6050 data
            self.magx,self.magy,self.magz = AK8963_conv()
            return np.array([self.accx,self.accy,self.accz,self.gyrox,self.gyroy,self.gyroz,self.magx,self.magy,self.magz])
       


    def calibrated_mpu9250(self):
        raw  = self.get_mpu9250_data()
         
        mpu_cal = np.zeros_like(raw)
        cal_rot_indicies = [[6,7],[7,8],[6,8]] # heading indices
        for i in range(3):   
            mpu_cal[i] = self.offsets[i][0]*raw[i]+ self.offsets[i][1]
        for i in range(3,6):
            mpu_cal[i] = raw[i] - self.offsets[i]
        for i in range(6,9):
            j = i-6
            mpu_cal[i] = raw[i] - self.offsets[cal_rot_indicies[j][0]]
        self.accx,self.accy,self.accz,self.gyrox,self.gyroy,self.gyroz,self.magx,self.magy,self.magz = mpu_cal
        return mpu_cal          
    
    

    def read_calibration_file(self):
        cal_offsets = np.array([[],[],[],0.0,0.0,0.0,[],[],[]], dtype = 'object')
        with open(self.file,'r',newline='') as csvfile:
            reader = csv.reader(csvfile,delimiter=',')
            iter_ii = 0
            for row in reader:
                if len(row)>2:
                    row_vals = [float(ii) for ii in row[int((len(row)/2)+1):]]
                    cal_offsets[iter_ii] = row_vals
                else:
                    cal_offsets[iter_ii] = float(row[1])
                iter_ii+=1
        return cal_offsets
            

    def telemetry(self):            
#        self.motors_voltage, self.motors_current = self.get_data_ina219(self.ina1)
#        sleep(0.020)
#        
#        self.rpi_voltage, self.rpi_current = self.get_data_ina219(self.ina2)
#        sleep(0.020)
        
        self.calibrated_mpu9250()
        #sleep(0.020)


        
