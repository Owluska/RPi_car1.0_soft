# -*- coding: utf-8 -*-
"""
Created on Wed Jan 27 11:40:58 2021

@author: User
"""
# if script doesn't work, check file path in console
import matplotlib.pyplot as plt
#import matplotlib.pyplot as plt
from gnss_ins_sim.sim import imu_model
from gnss_ins_sim.sim import ins_sim
import math
import numpy as np

D2R = math.pi/180
R2D = 180/math.pi
np.set_printoptions(precision=3)
motion_def_path = 'D:/!Phyton_Source/car_iekf/gnss-ins-sim/demo_motion_def_files/'
fs = 100.0          # IMU sample frequency
fs_mag = fs         # magnetometer sample frequency, not used for now

def test_path_gen():
    '''
    test only path generation in Sim.
    '''
    #### choose a built-in IMU model, typical for IMU381
    #imu_err = 'low-accuracy'
    #imu_err = 'mid-accuracy'

    imu_err = {
            # gyro bias, deg/hr
            'gyro_b': np.array([0.0014, 0.0014, 0.0014]),
            # gyro angle random walk, deg/rt-hr
            'gyro_arw': np.array([0.00016, 0.00016, 0.00016]),
            # gyro bias instability, deg/hr
            'gyro_b_stability': np.array([0.0, 0.0, 0.0]),
            # gyro bias instability correlation, sec.
            # set this to 'inf' to use a random walk model
            # set this to a positive real number to use a first-order Gauss-Markkov model
            'gyro_b_corr': np.array([np.Inf, np.Inf, np.Inf]),
            # accelerometer bias, m/s^2
            'accel_b': np.array([0.0e-3, 0.0e-3, 0.0e-3]),
            # accelerometer velocity random walk, m/s/rt-hr
            'accel_vrw': np.array([5.0e-6, 5.0e-6, 5.0e-6]),
            # accelerometer bias instability, m/s^2
            'accel_b_stability': np.array([0.60e-3, 0.60e-3, 0.80e-3]),
            # accelerometer bias instability correlation, sec. Similar to gyro_b_corr
            'accel_b_corr': np.array([np.Inf, np.Inf, np.Inf]),
            # magnetometer noise std, uT
            'mag_std': np.array([0.2, 0.2, 0.2])
          }
    # generate GPS and magnetometer data
    imu = imu_model.IMU(accuracy=imu_err, axis=9, gps=False)

    #### start simulation
    sim = ins_sim.Sim([fs, fs, fs_mag],
                      motion_def_path+"turn.csv",
                      ref_frame=1,
                      imu=imu,
                      mode=None,
                      env=None,
                      algorithm=None)
    sim.run()
    # plot data, 3d plot of reference positoin, 2d plots of gyro and accel
    sim.plot(['ref_att_euler', 'gyro', 'mag', 'accel'], opt={'ref_pos': '2d'})
    # save simulation data to files
    res = []
    res = sim.get_data(['time', 'ref_pos', 'ref_vel', 'ref_accel', 'ref_gyro', 'gyro', 'mag', 'accel'])
    return res
# if __name__ == '__main__':
imu_data = test_path_gen()

import pandas as pd
imu_df = pd.DataFrame()

imu_df.insert(0, "time", imu_data[0])

poss = imu_data[1].T
cols = ['rx', 'ry', 'rz']
lenght = len(cols)


for p, c, i in zip(poss, cols, range(lenght)):
    lc = i + 1
    imu_df.insert(lc, c, p - p.mean())

vels = imu_data[2].T
cols = ['rvx', 'rvy', 'rvz']
lenght = len(cols)
for p, c, i in zip(vels, cols, range(lenght)):
    lc = i + 1
    imu_df.insert(lc, c, p)
    

ref_accs = imu_data[2].T
cols = ['rax', 'ray', 'raz']
lenght = len(cols)
for p, c, i in zip(ref_accs, cols, range(lenght)):
    lc = i + 1
    imu_df.insert(lc, c, p)
    
ref_gyros = imu_data[2].T
cols = ['rgx', 'rgy', 'rgz']
lenght = len(cols)
for p, c, i in zip(ref_gyros, cols, range(lenght)):
    lc = i + 1
    imu_df.insert(lc, c, p)
    
gyros = imu_data[3][0].T * R2D
cols = ['gx', 'gy', 'gz']
lenght = len(cols)
for p, c, i in zip(gyros, cols, range(lenght)):
    lc = i + 1
    imu_df.insert(lc, c, p)
    
mags = imu_data[4][0].T
cols = ['magx', 'magy', 'magz']
lenght = len(cols)
for p, c, i in zip(mags, cols, range(lenght)):
    lc = i + 1
    imu_df.insert(lc, c, p)
    
accs = imu_data[5][0].T
cols = ['accx', 'accy', 'accz']
lenght = len(cols)
for p, c, i in zip(accs, cols, range(lenght)):
    lc = i + 1
    imu_df.insert(lc, c, p)
    
imu_df.to_excel("imu_model.xlsx")