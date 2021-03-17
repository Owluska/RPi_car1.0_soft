#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Mar 17 14:42:31 2021

@author: root
"""

from LIBRARY.US_mp_example import US
import numpy as np
import matplotlib.pyplot as plt
from time import time
#import multiprocessing
from multiprocessing import Pool, cpu_count, Process

US1_TRIG = 22
US1_ECHO = 17

US2_TRIG = 24
US2_ECHO = 23

back = US(trig = US1_TRIG, echo = US1_ECHO)
front = US(trig = US2_TRIG, echo = US2_ECHO)
#ds = np.array([[0]])
#ts = np.array([[0]])

#d = back.get_distance()
#
#t = .0
#dt = time()

n = cpu_count()
pool_size = 2
#pool = Pool(processes = 2)
pool = []

def US_pooling(US, stop):
#    t = .0
#    d = back.get_distance()
#    ds = np.array([[d]])
#    ts = np.array([[0]])
#    dt = time()
    while(1):
    #for i in range(100):

#        dt = time() - dt
##        _dt = dt
#        t += dt
        d = back.get_distance()
#        dt = time()
#        ds = np.append(ds, np.array([[d]]), axis = 0)
#        ts = np.append(ts, np.array([[t*1000]]), axis = 0)
        
        if d != None:
            print("{}:  {:.1f} cm".format(US, d))
#        if stop:
#            print("Stopping")
#            break

    
  
            
#while(1):
#
#    try:
#        dt = time() - dt
##        _dt = dt
#        t += dt
#        d = back.get_distance()
#        dt = time()
#        ds = np.append(ds, np.array([[d]]), axis = 0)
#        ts = np.append(ts, np.array([[t*1000]]), axis = 0)
#        
#        if d != None:
#            print("{:.1f} ms {:.1f} cm".format(t*1000, d))
#
#    except KeyboardInterrupt:
#        break
    
#plt.plot(ts, ds)
USs = [front, back]

stop = False
for i, us in zip(range(pool_size), USs):
    pool.append(Process(target = US_pooling, args=(us, stop,)))

for p in pool:
    p.daemon = True
    p.start()
while(1):
    try:
        pass
    except KeyboardInterrupt:
            stop = True
            break

for p in pool:
    p.terminate()
    p.close()
#pool.map(US_pooling, 2)    