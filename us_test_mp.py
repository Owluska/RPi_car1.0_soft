#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Mar 17 14:42:31 2021

@author: root
"""

from LIBRARY.rpi_US import US
import numpy as np
import matplotlib.pyplot as plt
from time import time, sleep

from multiprocessing import Pool, cpu_count, Process, Value, Manager

US1_TRIG = 22
US1_ECHO = 17

US2_TRIG = 24
US2_ECHO = 23

back = US(trig = US1_TRIG, echo = US1_ECHO)
front = US(trig = US2_TRIG, echo = US2_ECHO)


n = cpu_count()
pool_size = 2
#pool = Pool(processes = 2)
pool = []

def US_pooling(US, label, ret_value):
    while(1):
        try:
            d = US.get_distance()
            ret_value[label] = d

#            if d != None:
#                print("{}:  {:.1f} cm\n".format(label, d))
        
        except Exception:
            pass

    
  
            
#while(1):
#
#    try:
#        dt = time() - dt
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
#    
#plt.plot(ts, ds)


USs = [front, back]
labels = ['front', 'back']
dct = Manager().dict({l:0.0 for l in labels})

for i, us, l in zip(range(pool_size), USs, labels):
    pool.append(Process(target = US_pooling, args=(us, l, dct)))
for p in pool:
    p.daemon = True
    p.start()


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
  
        d = np.array([dct[l] for l in labels])
        ds = np.append(ds, d.reshape(1,2), axis = 0)
        ts = np.append(ts, np.array([[t]]), axis = 0)
        #print(dct)
        #sleep(0.05)
    except KeyboardInterrupt:
        break

for p in pool:
    p.join()
    p.close()

plt.plot(ts,ds)
plt.grid() 