# -*- coding: utf-8 -*-
"""
Created on Wed Jan 27 11:40:58 2021

@author: User
"""
# if script doesn't work, check file path in console
import matplotlib.pyplot as plt
#import matplotlib.pyplot as plt
# import sys
# sys.path.append('../gnss_ins_sim/gnss_ins_sim/')
from gnss_ins_sim.sim import imu_model
from gnss_ins_sim.sim import ins_sim
import math
import numpy as np

D2R = math.pi/180
R2D = 180/math.pi
np.set_printoptions(precision=3)
motion_def_path = 'demo_motion_def_files/'
fs = 100.0          # IMU sample frequency
fs_mag = fs         # magnetometer sample frequency, not used for now
fs_gps = fs/100
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
    gps_err = {'stdp': np.array([5.0, 5.0, 7.0]) * 0.2,
               'stdv': np.array([0.05, 0.05, 0.05]) * 1.0}
    odo_err = {'scale': 0.999,
               'stdv': 0.01}
    # generate GPS and magnetometer data
    imu = imu_model.IMU(accuracy=imu_err, axis=9, gps=True, gps_opt=gps_err,
                        odo=True, odo_opt=odo_err)

    #### start simulation
    sim = ins_sim.Sim([fs, fs_gps, fs_mag],
                      motion_def_path+"motion_def-Holland_tunnel.csv",
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
    res = sim.get_data(['time', 'gps_time', 'ref_gps', 'ref_pos', 'ref_vel', 'ref_accel', 'ref_gyro', 'odo', 'gyro', 'mag', 'accel'])
    return res
# if __name__ == '__main__':
imu_data = test_path_gen()

def parse_data(df, raw_data, labels, index):
    data = raw_data[index].T
    lenght = len(labels)
    #print(data)
    for d, c, i in zip(data, labels, range(lenght)):
        lc = i + 1
        df.insert(lc, c, d)

def parse_gdata(df, raw_data, labels, index):
    data = raw_data[index].T[0:3]
    lenght = len(labels)    
    for d, c, i in zip(data, labels, range(lenght)):
        lc = i + 1
        df.insert(lc, c, d - d[0])

def parse_sdata(df, raw_data, labels, index):
    data = raw_data[index][0].T
    lenght = len(labels)
    for d, c, i in zip(data, labels, range(lenght)):
        lc = i + 1
        df.insert(lc, c, d - d)
    

import pandas as pd

gps_df = pd.DataFrame()
gps_df.insert(0, "time", imu_data[1])
cols = ['gx', 'gy', 'gz']
parse_gdata(gps_df, imu_data, cols, 2)

imu_df = pd.DataFrame()
imu_df.insert(0, "time", imu_data[0])

cols = ['rx', 'ry', 'rz']
parse_data(imu_df, imu_data, cols, 3)
cols = ['rvx', 'rvy', 'rvz']
parse_data(imu_df, imu_data, cols, 4)
cols = ['rax', 'ray', 'raz']
parse_data(imu_df, imu_data, cols, 5)
cols = ['rgx', 'rgy', 'rgz']
parse_data(imu_df, imu_data, cols, 6)    

i = len(imu_df.columns)
imu_df.insert(i, "odom", imu_data[7][0])

cols = ['gyrox', 'gyroy', 'gyroz']
parse_sdata(imu_df, imu_data, cols, 8) 
cols = ['magx', 'magy', 'magz']
parse_sdata(imu_df, imu_data, cols, 9) 
cols = ['accx', 'accy', 'accz']
parse_sdata(imu_df, imu_data, cols, 10)    

    
imu_df.to_excel("imu_df.xlsx")
gps_df.to_excel("gps_df.xlsx")