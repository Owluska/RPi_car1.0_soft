# -*- coding: utf-8 -*-
"""
Created on Mon Feb  1 15:01:38 2021

@author: User
"""
# import numpy as np
# # import scipy as sc
from LIBRARY.rpi_car import rpi_movement
from LIBRARY.car_es_ekf import ekf
from LIBRARY.rpi_car_mag_calibration import write_calibration_file
from LIBRARY.rpi_telemetry import mb_telemetry
from LIBRARY.rotations import Quaternion, angle_normalize, rpy_jacobian_axis_angle
from LIBRARY.rpi_US_multi import US_multi

from time import time, sleep
import numpy as np
from math import pi, sqrt, sin, cos, atan2
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


def eulers_mag_acc(spec_force, mag, tilt_comp = True):
    l = mag.shape[0]
    
    eulers = np.zeros([l, 3])
    for f,m,i in zip(spec_force, mag, range(l)):
        ax = f[0]
        ay = f[1]
        az = f[2]
        
        roll = atan2(ay, az)
        pitch = atan2(-ax, sqrt(ay ** 2 + az **2))
        
        #m = angle_normalize(m)
        mx = m[0]
        my = m[1]
        mz = m[2]
        if tilt_comp:
        
            Mx = mx * cos(pitch) + mz * sin(pitch)
            My = mx * sin(roll) * sin(pitch) + my * cos(roll) - mz * sin(roll) * cos(pitch)
            yaw = atan2(My, Mx)
        else:
            yaw = atan2(-my, mx)
        eulers[i] = [roll, pitch, yaw]
    return eulers

def data_to_arrays(telemetry):
    w = np.array([[telemetry.gyrox,telemetry.gyroy, telemetry.gyroz]]) * telemetry.D2R
    f = np.array([[telemetry.accx,telemetry.accy, telemetry.accz]]) * telemetry.g
    m = np.array([[telemetry.magx,telemetry.magy, telemetry.magz]])
    return f,w,m


def plot_fig(data, label):
    fig = plt.figure(figsize = (10,10))
    plt.title(label)
    plt.plot(data)
    plt.grid() 
        
def turn_rand():
    r = np.random.randint(low = 0, high = 3)
    if r == 0:
        car.turn_left()
    elif r == 1:
        car.turn_center()
    elif r == 2:
        car.turn_right()

def circle_mov_script(mb, uss, uv_counter, US_threshold):
    if mb.motors_voltage != None:
        if(mb.motors_voltage < voltage_threshold):
            uv_counter += 1
            if uv_counter > 4:
                car.stop()
                car.turn_center()
                print("Undervoltage!!")
                return 'UV' 
    
    if uss.USs_out['front'] != None:        
        if(uss.USs_out['front'] < US_threshold):
                car.move_forward()
                turn_rand()
                print("Obstacle ahead!")
                sleep(0.2)
                return 'OA'
    
    if uss.USs_out['back'] != None:      
        if(uss.USs_out['back'] < US_threshold):
                car.move_backward()
                turn_rand()
                print("Obstacle behind!")
                sleep(0.2)
                return 'OB'
    else:
            car.move_backward()
            car.turn_right()
#                sleep(0.02)
#                self.car.stop()
#                sleep(0.01)
            uv_counter = 0
            return 'OK' 





car = rpi_movement()
car.init()


mb = mb_telemetry()
mb.init_all()

uss = US_multi()
uss.US_start()

voltage_threshold = 6.7
uv_counter = 0
state = circle_mov_script(mb, uss, uv_counter, voltage_threshold) 

sleep(1)

#toCalibrate = False 
#attempts = 3
#if toCalibrate:
#  write_calibration_file(car, auto = False, attempts = attempts)
  

#bias_m = np.array([206.818,  17.819,   9.807])
#bias_w = np.array([ 0.027, -0.041,  0.018])
#bias_f = np.array([-0.531, -0.486,  0.575])

var_m = np.array([5.774, 1.119, 1.466])
var_w = np.array([0.067, 0.107, 0.029])
var_f = np.array([1.962, 3.31 , 1.603])
es_var = np.array([2.797, 0.174, 0.675])



tmp = []
data = []

np.set_printoptions(precision=3)
np.set_printoptions(suppress=True)

mb.telemetry()
fs,ws,ms = data_to_arrays(mb)
#fs -= bias_f
#ws -= bias_w
#ms -= bias_m
es = np.array(eulers_mag_acc(fs, ms)).reshape(1,3)
dt = time()
ts = np.zeros([1,1])
dts = np.zeros([1,1])

kf = ekf()
kf.var_f = var_f
kf.var_w = var_w
kf.var_m = var_m

kf.N = kf.define_N()
kf.R = kf.define_R()
print("N:\n{}\nR\n\n{}".format(kf.N, kf.R))

kf.norm = True
if kf.norm:
    q_t0 = Quaternion(*ws[0]).normalize().to_numpy()
else:
    q_t0 = Quaternion(*ws[0]).to_numpy()
#kf.ROT =  Quaternion(*q_t0).to_mat()
kf.ROT =  np.eye(3)

p_cov_t0 = np.ones(9) * 0.01
ps_t0 = np.zeros([1,3])
vs_t0 = np.zeros([1,3])
kf.init_data(ps_t0, vs_t0, q_t0, p_cov_t0)
print("ROT\n:{}\n".format(kf.ROT))


np.set_printoptions(precision=3)
np.set_printoptions(suppress=True)
useKF = True

sensors_data = np.zeros([1,9])
sdata = 'Time x y z'
print(sdata)


toMove = True
counter = 0
while(1):
    if counter > 200:
        break
    try:
        dt = time() - dt
        dts = np.append(dts, np.array([[dt]]), axis = 0)

        mb.time += dt
        ts = np.append(dts, np.array([[mb.time]]), axis = 0)
        dt = time()

        
        
        mb.telemetry()
        f,w,m = data_to_arrays(mb)
#        f -= bias_f
#        w -= bias_w
#        m -= bias_m
        

        e = np.array(eulers_mag_acc(f, m, tilt_comp = False)).reshape(1,3)
        es = np.append(es, e, axis = 0)
        ws = np.append(ws, w.reshape(1,3), axis = 0)
        fs = np.append(fs, f.reshape(1,3), axis = 0)
        ms = np.append(ms, m.reshape(1,3), axis = 0)
        print(kf.p_est[counter])
        
        if useKF:
            sensors_data[0, 6:] = e
            kf.update(fs[0], ws[0], dts[counter, 0], counter,
                      useFilter = useKF,
                      sensors_data = sensors_data)
        else:
            kf.update(f[0], w[0], ts[counter, 0], counter, useFilter = useKF, sensors_data = sensors_data) 
 
        if toMove:
            try:
                state = circle_mov_script(mb, uss, uv_counter, voltage_threshold) 
            except Exception as e:
                template = "An exception of type {0} occured. Arguments:\n{1!r}"
                message = template.format(type(e).__name__, e.args)
                print(message)

        counter += 1


    except KeyboardInterrupt:
        template = "An exception of type {0} occured. Arguments:\n{1!r}"
        message = template.format(type(e).__name__, e.args)
        print(message)
        break


uss.USs_stop()
car.turn_center()
car.stop()
     

fig = plt.figure(figsize = (10,10))
plt.title("pos")
plt.plot(kf.p_est[:, :2])
plt.grid()


fig = plt.figure(figsize = (10,10))
plt.title("vels")
plt.plot(kf.v_est[:, :2])
plt.grid()