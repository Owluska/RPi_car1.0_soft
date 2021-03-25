#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Fri Mar 19 10:40:35 2021

@author: root
"""
from multiprocessing import Process, Manager
from LIBRARY.rpi_US import US

US1_TRIG = 22
US1_ECHO = 17

US2_TRIG = 24
US2_ECHO = 23

back = US(trig = US1_TRIG, echo = US1_ECHO)
front = US(trig = US2_TRIG, echo = US2_ECHO)
back.setup_US_ports()
front.setup_US_ports()

#print(back,front)


class US_multi():
    def __init__(self, USs = [US(trig = US1_TRIG, echo = US1_ECHO), US(trig = US2_TRIG, echo = US2_ECHO)], labels = ['back', 'front']):
        self.USs = USs
        self.USs_labels = labels
        self.US_pools = []
        self.USs_out = Manager().dict({l:0.0 for l in self.USs_labels}) 
        self.toSleep = 0.2
        
        
    def US_pooling(self, US, label):
        while(1):
            try:
                d = US.get_distance()
                #print(self.USs_out)
                self.USs_out[label] = d        
            except Exception:
                return

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
      