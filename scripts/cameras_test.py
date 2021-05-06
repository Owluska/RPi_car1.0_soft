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

orb = cv2.ORB_create()

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
    #image resize scale in percents
    scale = 50
    width = int(frame1.shape[1] * scale/100)
    height = int(frame1.shape[0] * scale/100)
    dsize = (width, height)
    rframe1 = cv2.resize(frame1, dsize)
    rframe2 = cv2.resize(frame2, dsize)
    #original frame to grayscale    
    gray1 = cv2.cvtColor(rframe1, cv2.COLOR_BGR2GRAY)
    gray2 = cv2.cvtColor(rframe2, cv2.COLOR_BGR2GRAY)
    #show original frame
#    cv2.imshow('1', gray1)
#    cv2.imshow('2', gray2)
    #find the keypoints and descriptions with SIFT detector
    kp1, des1 = orb.detectAndCompute(gray1, None)
    kp2, des2 = orb.detectAndCompute(gray2, None)
    #create BF matcher object
    bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck = True)
    matches = bf.match(des1, des2)
    #sort matches in order of their distance
    matches = sorted(matches, key = lambda x:x.distance)
    #draw first ten matches
    frame3 = cv2.drawMatches(gray1, kp1, gray2, kp2, matches[:10], None, flags=2)
    cv2.imshow('matches', frame3)
    
    key = cv2.waitKey(1) & 0xFF
    if key == ord('q') or key == ord('Q'):
        break


cap1.release()
cap2.release()
cv2.destroyAllWindows()