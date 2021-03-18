#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Mar 17 14:42:31 2021

@author: root
"""

from LIBRARY.rpi_US import US
from LIBRARY.rpi_car import rpi_movement
from LIBRARY.rpi_telemetry import mb_telemetry


import numpy as np
import matplotlib.pyplot as plt
from time import time, sleep

from multiprocessing import cpu_count, Process, Manager

US1_TRIG = 22
US1_ECHO = 17

US2_TRIG = 24
US2_ECHO = 23

back = US(trig = US1_TRIG, echo = US1_ECHO)
front = US(trig = US2_TRIG, echo = US2_ECHO)



#n = cpu_count()

#pool = Pool(processes = 2)
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
                self.turn_rand(car)
                print("Obstacle ahead!")
                sleep(0.1)
                return 'OA'
        elif(self.USs_out['back'] < self.US_threshold):
                self.car.move_backward()
                self.turn_rand(car)
                print("Obstacle behind!")
                sleep(0.1)
                return 'OB'
        else:
                car.move_backward()
                car.turn_center()
                sleep(0.02)
                car.stop()
                sleep(0.01)
                uv_counter = 0
                
                print("k")
                return 'OK'       
            
    def stop_mvmnt(self, car):
        self.car.stop()
        self.car.turn_center()
        self.USs_stop()
        
        
    def data_to_arrays(telemetry):
        g = -9.81
        w = np.array([[telemetry.gyrox,telemetry.gyroy, telemetry.gyroz]])
        f = np.array([[telemetry.accx,telemetry.accy, telemetry.accz]]) * g
        m = np.array([[telemetry.magx,telemetry.magy, telemetry.magz]])
        return f,w,m  
            

#def random_0_4():
#    return np.random.randint(low = 0, high = 4)


        

#USs = [front, back]
#labels = ['front', 'back']
        
car = rpi_movement()
car.init()
mb = mb_telemetry()
mb.init_all()
mb.telemetry()
    
mvmnt = random_mvmnt(car, mb)
mvmnt.US_start()

#uss_out, uss_pool = US_start(USs, labels)
#voltage_threshold = 6.7
#uss_threshold = 10

ds = np.array([[0., 0.]])
ts = np.array([[0]])
d = back.get_distance()
t = .0
dt = time()

while(1):
    try:
        dt = time() - dt
        t += dt
        dt = time()
  
        d = np.array([mvmnt.USs_out[l] for l in mvmnt.USs_labels])
        ds = np.append(ds, d.reshape(1,2), axis = 0)
        ts = np.append(ts, np.array([[t]]), axis = 0)
#        sleep(0.05)
        try: 
            state = mvmnt.random_mvmnt_obstacle_avoiding()
            print(state)
#            if(mb.motors_voltage < voltage_threshold):
#                uv_counter += 1
#                if uv_counter > 4:
#                    motors = car.stop()
#                    rul = car.turn_center()
#                    print("Undervoltage!!")
#                    break 
#            elif(uss_out['front'] < 20):
#                    motors = car.move_forward()
#                    turn_rand(car)
#                    print("Obstacle ahead!")
#                    sleep(0.1)
#            
#            elif(uss_out['back'] < 20):
#                    motors = car.move_backward()
#                    turn_rand(car)
#                    print("Obstacle behind!")
#                    sleep(0.1)
#            else:
#                    motors = car.move_backward()
#                    rul = car.turn_center()
#                    sleep(0.02)
#                    motors = car.stop()
#                    sleep(0.01)
#                    uv_counter = 0
            
        except Exception:
            pass
    except KeyboardInterrupt:
        break

        
#USs_stop(uss_pool)
#stop_mvmnt(car)
mvmnt.stop_mvmnt(car)
plt.plot(ts,ds)
plt.grid() 