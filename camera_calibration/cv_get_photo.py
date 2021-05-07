#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Fri May  7 09:33:00 2021

@author: root
"""

import cv2
import numpy as np
#0 - right camera, 2 - left camera
cap = cv2.VideoCapture(2)
pref = 'im_left'


n = 0

while(True):
    
    ret, frame = cap.read()
    
    if not ret:
        cap.release()
        cv2.destroyAllWindows()
        print('Error capturing camera1. Breaking loop')
        break
    
    
    #cv2.imshow('frame', frame)

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    inverted = np.array(256 - gray, dtype = 'uint8')
    cv2.imshow('', inverted)
    #ret, corners = cv2.findChessboardCorners(frame, (5, 5), None)

    
    key = cv2.waitKey(1) & 0xFF
    if key == ord('s') or key == ord('S'):
#        if not ret:
#                print("Fault")
#                print(corners)
#                continue
        n += 1
        name = "{}{}.jpg".format(pref,n)
        cv2.imwrite(name, frame)
    if key == ord('q') or key == ord('Q'):
        break


cap.release()
cv2.destroyAllWindows()
