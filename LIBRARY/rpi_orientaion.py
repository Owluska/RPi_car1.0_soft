#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sat Dec 12 15:56:29 2020

@author: root
"""

import math
import matplotlib.pyplot as plt
from scipy import integrate

class rpi_orientation:
  
    def normalize_3axis(self, x, y, z):
        n = math.sqrt(x**2 + y**2 + z**2)
        x /= n
        y /= n
        z /= n
        return x, y, z
    
    def yaw_from_mags(self, mx, my, mz, norm = True):
        
        if norm:
            mx, my, mz = self.normalize_3axis(mx, my, mz)
        
        yaw = math.atan2(my, mx) * 180/math.pi
        
        yaw = round(yaw, 3)
        return yaw
    
    
    def roll_from_acc(self, board):
        ax = board.accx
        ay = board.accy
        az = board.accz - 1
        mu = 0.01
        if az > 0:
            roll = math.atan2(ay, math.sqrt(az**2 + mu * ax**2))
        elif az < 0:
            roll = math.atan2(ay, -math.sqrt(az**2 + mu*ax**2))
    #    print("roll:", roll)
        return roll
    
    def pitch_from_acc(self, board):
        ax = board.accx
        ay = board.accy
        az = board.accz - 1
        
        pitch = math.atan2(-ax, math.sqrt(ay**2 + az**2))
    #    print("pitch:", pitch)
        return pitch
        
    def yaw_from_mags_tilt_compensate(self, board, norm = True):
        
        roll = self.roll_from_acc(board)
        pitch = self.pitch_from_acc(board)
        
        mx = board.magx
        my = board.magy
        mz = board.magz
        
        # if norm:
        #     mx, my, mz = normalize_3axis(mx, my, mz)
        
        My = my*math.cos(roll) + mz*math.sin(roll)
        Mx = mx*math.cos(pitch)  + my*math.sin(roll)*math.sin(pitch) - mz*math.sin(pitch)*math.cos(roll)
     
        yaw = self.yaw_from_mags(Mx, My, mz, norm = norm)
        return yaw
                        
    def yaw_from_gyro_euler(self, previous_yaw, board):
        dt = board.time  
    
        yaw = previous_yaw + board.gyroz*dt
    
        if abs(yaw) > 180:
            yaw = -180 + yaw % (360)
        yaw = round(yaw,3)
        return yaw
    
    def yaw_from_gyros(gyros, times, init):
    
        yaw = integrate.cumtrapz(gyros, times, initial = init)[-1]
    
        if abs(yaw) > 180:
            yaw = -180 + yaw % (360)
        yaw = round(yaw,3)
        return yaw
    
        
    def plot_data_n_labels(self, x, ys, title = '', xlabel = '',
                           ylabel = '', legend =None):
        fig = plt.figure(figsize = (10,10))
        
        for y in ys:
            plt.plot(x,y)
        plt.title(title)
        plt.xlabel(xlabel)
        plt.ylabel(ylabel)
        plt.grid()
        
        if legend != None:
            plt.legend(legend)
