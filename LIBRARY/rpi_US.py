#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Mar 17 14:31:39 2021

@author: root
"""
import wiringpi
from time import time, sleep
class US:
    def __init__(self, trig = 24, echo = 23):
        self.US = None
        self.US_TRIG = trig
        self.US_ECHO = echo
        self.T = 25
        self.sound_vel=(331.5+0.6*self.T)
        self.us = 1e-6
        self.ms = 1e-3
        self.timeout = 10 * self.ms
        self.dist = None        

    def setup_US_ports(self):
        wiringpi.wiringPiSetupGpio()
        wiringpi.pinMode(self.US_ECHO, wiringpi.GPIO.INPUT)
    
        wiringpi.pinMode(self.US_TRIG, wiringpi.GPIO.OUTPUT)
        wiringpi.digitalWrite(self.US_TRIG, wiringpi.GPIO.LOW)

    def get_distance(self):
        sleep(0.38)
#        sleep(0.05)
        pulse_start = 0
        pulse_end = 0
#        if US == 1:
#            triger = self.US1_TRIG
#            echo = self.US1_ECHO            
#        elif US == 2:
#            triger = self.US2_TRIG
#            echo = self.US2_ECHO
        
        wiringpi.digitalWrite(self.US_TRIG, wiringpi.GPIO.LOW)
        sleep(0.05)
        
        wiringpi.digitalWrite(self.US_TRIG, wiringpi.GPIO.HIGH)
        sleep(10*self.us)
        wiringpi.digitalWrite(self.US_TRIG, wiringpi.GPIO.LOW)
        
        start_time = time()
        while(wiringpi.digitalRead(self.US_ECHO) == wiringpi.GPIO.LOW):
            pulse_start = time()
            if(pulse_start - start_time > self.timeout):
                break
            
        start_time = time()
        while(wiringpi.digitalRead(self.US_ECHO) == wiringpi.GPIO.HIGH):
            pulse_end = time()
            if(pulse_end - start_time > self.timeout):
                break
        
     
        distance = self.sound_vel * (pulse_end - pulse_start) *0.5*100
        distance = abs(round(distance, 2))
        
        #print(distance)
        if distance < 0 or distance > self.sound_vel * (self.timeout) * 100:
                distance = None            
        self.dist = distance
        return distance        