# -*- coding: utf-8 -*-
"""
Created on Mon Feb  1 15:01:38 2021

@author: User
"""
# import numpy as np
# # import scipy as sc
from LIBRARY.rpi_car import rpi_movement
from LIBRARY.rpi_car_mag_calibration import write_calibration_file
from LIBRARY.rpi_telemetry import mb_telemetry
from LIBRARY.car_iekf import car_iekf
from time import time
import numpy as np
import matplotlib.pyplot as plt

def plot_data_n_labels(x, ys, title = '', xlabel = '',
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

def measure_bias_n_std(mb, points = 10):
    gxs,gys, gzs, axs, ays, azs = [],[],[],[],[],[]
    biases = np.empty((1, 6), dtype = 'double')
    stds = biases.copy()
    print("Measuring bias and std, amount of poinst: ", points)
    for i in range(points):
        end = ' ' * i
        print(i, end = end)
        gxs.append(mb.gyrox)
        gys.append(mb.gyroy)
        gzs.append(mb.gyroz)
        
        axs.append(mb.accx)
        ays.append(mb.accy)
        azs.append(mb.accz)
        
    biases[:, :3] = np.array([np.mean(gxs), np.mean(gys), np.mean(gzs)])
    biases[:, 3:] = np.array([np.mean(axs), np.mean(ays), np.mean(azs)])
    
    stds[:, :3] = np.array([np.std(gxs), np.std(gys), np.std(gzs)])
    stds[:, 3:] = np.array([np.std(axs), np.std(ays), np.std(azs)])
    return biases, stds

car = rpi_movement()
car.init()
mb = mb_telemetry()
mb.init_all()


toCalibrate = False 
attempts = 3
if toCalibrate:
  write_calibration_file(car, auto = False, attempts = attempts)
  

mb.read_mag_calibration_file()
print("Magnetometer caibration:")    
print(mb.magx_offset, mb.magx_scale, mb.magy_offset, mb.magy_scale, mb.magz_offset, mb.magz_scale)

dt = time()
g = 9.780318


sdata = 'Time x y z'

# obstacle_threshold = 15
voltage_threshold = 6.7
uv_counter = 0

ts, ps = [], []

biases, stds = measure_bias_n_std(mb)
    
gyros = np.array([mb.gyrox, mb.gyroy, mb.gyroz])
accs = np.array([mb.accx, mb.accy, mb.accz])
kf = car_iekf(gyros, accs,
              gyro_bias= biases[:,:3], acc_bias=biases[:,3:])    

print(sdata)
while(1):
    try:
        dt = time() - dt
        mb.time += dt
        dt = time()
        

        
        mb.telemetry()
        gyros = np.array([mb.gyrox, mb.gyroy, mb.gyroz])
        accs = np.array([mb.accx, mb.accy, mb.accz])
        ts.append(round(mb.time, 3))
        
        kf.propagate(gyros, accs, dt)
        kf.update(gyros, accs, dt)
        ps.append(kf.p)

        ps.append(kf.x[6])

        sdata = "{:.3f} {:.3f} {:.3f} {:.3f}".format(ts, kf.x[6][0][0], kf.x[6][0][1], kf.x[6][0][2]) 
        try:       
            if(mb.motors_voltage < voltage_threshold):
                uv_counter += 1
                if uv_counter > 4:
                    motors = car.stop()
                    rul = car.turn_center()
                    print("Undervoltage!!")
                    break
            else:
                motors = car.move_forward()
                rul = car.turn_right()
                uv_counter = 0
                print(sdata)
        
        
        except Exception:
            pass

        


    except KeyboardInterrupt:
        car.stop()
        car.turn_center()
        x = [ps[i][0][0] for i in range(len(ps))]
        y = [ps[i][0][1] for i in range(len(ps))]
        z = [ps[i][0][2] for i in range(len(ps))]
        
        plt.plot(ts, x)
        plt.plot(ts, y)
        plt.plot(ts, z)