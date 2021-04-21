#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Apr 20 11:48:30 2021

@author: root
"""

import cv2
from time import sleep


cap1 = cv2.VideoCapture(0)
cap2 = cv2.VideoCapture(2)


while(True):
    
    ret1, frame1 = cap1.read()
    ret2, frame2 = cap2.read()
    
    if not ret1:
        cap1.release()
        cv2.destroyAllWindows()
        print('Error capturing camera1. Breaking loop')
        break
    
    if not ret2:
        cap2.release()
        cv2.destroyAllWindows()
        print('Error capturing camera2. Breaking loop')
        break    
    cv2.imshow('1', frame1)
    cv2.imshow('2', frame2)
    
    key = cv2.waitKey(1) & 0xFF
    if key == ord('q') or key == ord('Q'):
        break


cap1.release()
cap2.release()
cv2.destroyAllWindows()