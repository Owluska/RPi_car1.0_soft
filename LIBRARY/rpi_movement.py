#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Fri Mar 19 10:40:35 2021

@author: root
"""
from multiprocessing import Process, Manager
from LIBRARY.rpi_US import US
import numpy as np
from time import sleep

US1_TRIG = 22
US1_ECHO = 17

US2_TRIG = 24
US2_ECHO = 23

back = US(trig = US1_TRIG, echo = US1_ECHO)
front = US(trig = US2_TRIG, echo = US2_ECHO)

class random_mvmnt():
    def __init__(self, car, mb, USs = [US(trig = US1_TRIG, echo = US1_ECHO), US(trig = US2_TRIG, echo = US2_ECHO)], labels = ['back', 'front']):
        self.USs = USs
        self.USs_labels = labels
        self.US_pools = []
        self.USs_out = Manager().dict({l:0.0 for l in self.USs_labels}) 
        self.voltage_threshold = 7.6
        self.US_threshold = 10
        self.car = car
        self.mb = mb
        
        
    def US_pooling(self, US, label):
        while(1):
            try:
                d = US.get_distance()
                #print(self.USs_out)
                self.USs_out[label] = d        
            except Exception:
                pass

    def US_start(self):
        pool_size = len(self.USs)
        #self.USS_out = Manager().dict({l:0.0 for l in self.USS_labels})    
        for i, us, l in zip(range(pool_size), self.USs, self.USs_labels):
            self.US_pools.append(Process(target = self.US_pooling, args=(us, l,)))
        for p in self.US_pools:
            p.daemon = False
            p.start()
#        return dct, pool
    
    def USs_stop(self):
        for p in self.US_pools:
            p.join()
            p.close()    
    
    def turn_rand(self):
        r = np.random.randint(low = 0, high = 9)
        if r == 0:
            self.car.turn_left()
        elif r == 3:
            self.car.turn_center()
        elif r == 6:
            self.car.turn_right()
    
    def random_mvmnt_obstacle_avoiding(self):        
        uv_counter = 0
        if(self.mb.motors_voltage < self.voltage_threshold):
            uv_counter += 1
            if uv_counter > 4:
                self.car.stop()
                self.car.turn_center()
                print("Undervoltage!!")
                return 'UV' 
        elif(self.USs_out['front'] < self.US_threshold):
                self.car.move_forward()
                self.turn_rand()
                print("Obstacle ahead!")
                sleep(0.1)
                return 'OA'
        elif(self.USs_out['back'] < self.US_threshold):
                self.car.move_backward()
                self.turn_rand()
                print("Obstacle behind!")
                sleep(0.1)
                return 'OB'
        else:
                self.car.move_backward()
                self.car.turn_center()
                sleep(0.02)
                self.car.stop()
                sleep(0.01)
                uv_counter = 0
                return 'OK'       
            
    def stop_mvmnt(self, car):
        self.car.stop()
        self.car.turn_center()
        self.USs_stop()