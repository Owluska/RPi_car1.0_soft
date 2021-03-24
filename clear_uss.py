#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Mar 24 10:51:58 2021

@author: root
"""
from LIBRARY.rpi_US_multi import US_multi
import numpy as np
from time import sleep

uss = US_multi()
uss.US_start()

while(1):
    try:
        d = np.array([uss.USs_out[l] for l in uss.USs_labels])
        print(d)
        sleep(0.1)
    except KeyboardInterrupt:
        break


uss.USs_stop()