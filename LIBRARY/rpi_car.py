#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sat Dec 12 15:34:02 2020

@author: root
"""
import wiringpi

class rpi_movement:
    
    def __init__(self):
        self.RUL = 12
        self.MOTOR_A = 6  
        self.MOTOR_B = 26

        self.RUL_CENTER = 150
        self.RUL_RIGHT = 200
        self.RUL_LEFT = 100
    
    def init(self):
        wiringpi.wiringPiSetupGpio()
    
        wiringpi.pinMode(self.MOTOR_A, wiringpi.GPIO.OUTPUT)
        wiringpi.pinMode(self.MOTOR_B, wiringpi.GPIO.OUTPUT)

        wiringpi.pullUpDnControl(self.MOTOR_A, wiringpi.GPIO.PUD_UP)
        wiringpi.pullUpDnControl(self.MOTOR_B, wiringpi.GPIO.PUD_UP)
    
        wiringpi.pinMode(self.RUL, wiringpi.GPIO.PWM_OUTPUT)
    
        wiringpi.pwmSetMode(wiringpi.GPIO.PWM_MODE_MS)
        wiringpi.pwmSetClock(192)
        wiringpi.pwmSetRange(2000)
        
    def stop(self):
        wiringpi.digitalWrite(self.MOTOR_A, wiringpi.GPIO.HIGH)
        wiringpi.digitalWrite(self.MOTOR_B, wiringpi.GPIO.HIGH)
        return 'S'
        
    def move_forward(self):
        wiringpi.digitalWrite(self.MOTOR_A, wiringpi.GPIO.LOW)
        wiringpi.digitalWrite(self.MOTOR_B, wiringpi.GPIO.HIGH)
        return 'F'
        
    def move_backward(self):
        wiringpi.digitalWrite(self.MOTOR_A, wiringpi.GPIO.HIGH)
        wiringpi.digitalWrite(self.MOTOR_B, wiringpi.GPIO.LOW)
        return 'B'
        
    def turn_right(self):
        wiringpi.pwmWrite(self.RUL, self.RUL_RIGHT)
        return 'R'
        
    def turn_left(self):
        wiringpi.pwmWrite(self.RUL, self.RUL_LEFT)
        return 'L'
    
    def turn_center(self):
        wiringpi.pwmWrite(self.RUL, self.RUL_CENTER)
        return 'C'