# -*- coding: utf-8 -*-
"""
Created on Mon Feb  1 15:01:38 2021

@author: User
"""
# import numpy as np
# # import scipy as sc
from LIBRARY.rpi_car import rpi_movement
from LIBRARY.car_es_ekf import ekf
#from LIBRARY.rpi_car_mag_calibration import write_calibration_file
from LIBRARY.rpi_telemetry import mb_telemetry
from LIBRARY.rotations import Quaternion, angle_normalize
from LIBRARY.rpi_US_multi import US_multi

from time import time, sleep
import numpy as np
from math import sqrt, sin, cos, atan2
import matplotlib.pyplot as plt

#def plot_data_n_labels(x, ys, title = '', xlabel = '',
#                       ylabel = '', legend =None):
#    fig = plt.figure(figsize = (10,10))
#    
#    for y in ys:
#        plt.plot(x,y)
#    plt.title(title)
#    plt.xlabel(xlabel)
#    plt.ylabel(ylabel)
#    plt.grid()
#    
#    if legend != None:
#        plt.legend(legend)

#data processing n plot
def eulers_mag_acc(spec_force, mag, tilt_comp = True):
    l = mag.shape[0]
    
    eulers = np.zeros([l, 3])
    for f,m,i in zip(spec_force, mag, range(l)):
        ax = f[0]
        ay = f[1]
        az = f[2]
        
        roll = atan2(ay, sqrt(ay ** 2 + az **2))
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
#    f = np.array([angle_normalize(fi) for fi in f])
#    m = np.array([angle_normalize(mi) for mi in m])
#    w = np.array([angle_normalize(wi) for wi in w])
    return f,w,m


def plot_fig(data, label, legend = ['x','y','z']):
    fig = plt.figure(figsize = (10,10))
    plt.title(label)
    plt.plot(data)
    plt.legend(legend)
    plt.grid() 

def average_measurments(samples, counter_value, raw, averaged, atype = 'mean'):        
        if counter_value % samples == 0:
            i = counter_value - samples
            if atype == 'average':
                weights = np.power(10, np.abs(raw[i:, :])*-10)
                d  = np.average(raw[i:, :], axis = 0, weights = weights)
            elif atype == 'median':
                d = np.median(raw[i:, :], axis = 0)
            else:
                d = np.mean(raw[i:, :], axis = 0)
            averaged = np.append(averaged, d.reshape(1,3), axis = 0)
            return averaged
        else:
            return averaged

def average_measurment(samples, counter_value, raw, atype = 'mean'):        
            i = counter_value - samples
            if atype == 'average':
                weights = np.power(10, np.abs(raw[i:, :])*-10)
                d  = np.average(raw[i:, :], axis = 0, weights = weights)
            elif atype == 'median':
                d = np.median(raw[i:, :], axis = 0)
            else:
                d = np.mean(raw[i:, :], axis = 0)
            return d
#movment
def turn_rand():
    r = np.random.randint(low = 0, high = 3)
    if r == 0:
        return car.turn_left()
    elif r == 1:
         return  car.turn_center()
    elif r == 2:
         return  car.turn_right()
        


def check_US(USs, us, labels = ['left', 'right'], US_threshold1 = 30):

    if USs.USs_out[labels[us]] == None:
        return 'us_' + labels[us] + '_none'
    elif USs.USs_out[labels[us]] < US_threshold1:
        return 'obstacle_' + labels[us]
    else:
        return 'us_' + labels[us] + '_ok'

def check_voltage(mb, voltage_threshold = 7.0):
    if mb.motors_voltage == None:
        return 'voltm_none'
    elif(mb.motors_voltage < voltage_threshold):
        return 'undervoltage'
    else:
        return 'voltage_ok'

def fromUS_mov_script(mb, uss, labels, uv_counter):
    template = "{}:{}:{}:{}:{}"
    keys = ['motors_power', 'left_us', 'right_us', 'velocity_state', 'rul_pos']
    s_dic = {k:'' for k in keys}
    s_dic['motors_power'] = check_voltage(mb)
    s_dic['left_us'] = check_US(uss, 0, labels)
    s_dic['right_us'] = check_US(uss, 1, labels)
    
    if s_dic['motors_power'] == 'undervoltage':
        uv_counter += 1
        if uv_counter > 4:
            s_dic['velocity_state'] = car.stop()
            s_dic['rul_pos'] = car.turn_center()
            s_dic['motors_power'] = 'undervoltage'
        else:
            s_dic['motors_power'] = 'voltage_ok'
        
    if s_dic['left_us'].find('obstacle') != -1\
    or s_dic['right_us'].find('obstacle') != -1:     
        s_dic['velocity_state'] = car.move_forward()
        if s_dic['rul_pos'] == 'R' or s_dic['rul_pos'] == 'C':
            s_dic['rul_pos'] = car.turn_left()
        elif s_dic['rul_pos'] == 'L':
            s_dic['rul_pos'] = car.turn_right()
        sleep(0.5)
        uv_counter = 0
    else:
        s_dic['velocity_state'] = car.move_backward()
        s_dic['rul_pos'] = turn_rand()
        sleep(0.05)
#                self.car.stop()
#                sleep(0.01)
        uv_counter = 0
    return template.format(*s_dic.values()), s_dic




#motors 
car = rpi_movement()
car.init()

#telemetry
mb = mb_telemetry()
mb.init_all()
labels = ['left', 'right']
uss = US_multi(labels = labels)
uss.US_start()

#starts movemtnt before kalman init
uv_counter = 0
state, dic = fromUS_mov_script(mb, uss, labels, uv_counter) 

sleep(2)

#sensors variances
var_m = np.array([5.774, 1.119, 1.466])
var_w = np.array([0.067, 0.107, 0.029])
var_f = np.array([1.962, 3.31 , 1.603])
es_var = np.array([2.797, 0.174, 0.675])

#init data
mb.telemetry()
fs,ws,ms = data_to_arrays(mb)
_fs = np.copy(fs)
_ws = np.copy(ws)
#fs -= bias_f
#ws -= bias_w
#ms -= bias_m
es = np.array(eulers_mag_acc(fs, ms)).reshape(1,3)
t = time()
ts = np.zeros([1,1])
dts = np.zeros([1,1])
sensors_data = np.zeros([1,9])
#create n init kalman
kf = ekf()
kf.var_f = var_f
kf.var_w = var_w
kf.var_m = var_m

kf.N = kf.define_N()
kf.R = kf.define_R()
print("N:\n{}\nR\n\n{}".format(kf.N, kf.R))

kf.norm = True
if kf.norm:
    q_t0 = Quaternion(axis_angle=ws[0]).normalize().to_numpy()
else:
    q_t0 = Quaternion(axis_angle = ws[0]).to_numpy()
#kf.ROT =  Quaternion(*q_t0).to_mat()
kf.ROT =  np.eye(3)

p_cov_t0 = np.ones(9) * 0.01
ps_t0 = np.zeros([1,3])
vs_t0 = np.zeros([1,3])
kf.init_data(ps_t0, vs_t0, q_t0, p_cov_t0)

print("ROT\n:{}\n".format(kf.ROT))

#set print options
np.set_printoptions(precision=3)
np.set_printoptions(suppress=True)
# Use correction?
useKF = False
#data label
#sdata = 'Time x y z'
templ = "time: {:.2f}s pos: {:.1f}m {:.1f}m {:.1f}m\t"
leg = ['x', 'y', 'z']
#print(sdata)

#move?
toMove = True
counter = 0
samples = 1 
av_type = 'mean'
k = 0
while(1):
    if counter > 500:
        break
    try:
        dt = time() - t
        t = time()
        
        mb.time += dt

        

#        d = np.array([uss.USs_out[l] for l in uss.USs_labels])
#        print(d)
        try:
            mb.telemetry()
        except Exception as e:
            template = "An exception of type {0} occured. Arguments:\n{1!r}"
            message = template.format(type(e).__name__, e.args)
            print(message)
            
        f,w,m = data_to_arrays(mb)
#        f -= bias_f
#        w -= bias_w
#        m -= bias_m
        
        ts = np.append(ts, np.array([[mb.time]]), axis = 0)
        dts = np.append(dts, np.array([[dt]]), axis = 0)
        e = np.array(eulers_mag_acc(f, m, tilt_comp = False)).reshape(1,3)
        es = np.append(es, e, axis = 0)
        ws = np.append(ws, w.reshape(1,3), axis = 0)
        fs = np.append(fs, f.reshape(1,3), axis = 0)
        ms = np.append(ms, m.reshape(1,3), axis = 0)
        
        _fs = average_measurments(samples = samples, counter_value = counter, raw = fs, averaged = _fs, atype=av_type)
        _ws = average_measurments(samples = samples, counter_value = counter, raw = ws, averaged = _ws, atype=av_type)
        

        print(templ.format(dt, *kf.p_est[k]), end = '')
        if counter % samples == 0:
            _f = average_measurment(samples = samples, counter_value = counter, raw = fs, atype=av_type)
            _w = average_measurment(samples = samples, counter_value = counter, raw = ws, atype=av_type)
            if dic['velocity_state'] == 'S':
                kf.v_est[k] = np.zeros(3)
                kf.a[k] = np.zeros(3)
                #print('standing')
            if useKF:
                sensors_data[0, 6:] = e
                kf.update(_f, _w, dt, k,
                          useFilter = useKF,
                          sensors_data = sensors_data)
            else:
                kf.update(_f, _w, dt, k, useFilter = useKF, sensors_data = sensors_data) 
            k += 1
 
        if toMove:
            try:
                state, dic = fromUS_mov_script(mb, uss, labels, uv_counter)
                print(state)
            except Exception as e:
                template = "An exception of type {0} occured. Arguments:\n{1!r}"
                message = template.format(type(e).__name__, e.args)
                print(message)

        counter += 1


    except KeyboardInterrupt:
#        template = "An exception of type {0} occured. Arguments:\n{1!r}"
#        message = template.format(type(e).__name__, e.args)
#        print(message)
        uss.USs_stop()
        car.turn_center()
        car.stop()
        break


uss.USs_stop()
car.turn_center()
car.stop()
print("Summ time: {:.3f} s".format(np.sum(dts)))     
plot_fig(kf.p_est[:, :2], 'pos')
plot_fig(kf.v_est[:, :2], 'vel')
plot_fig(kf.a, 'acc')

fig = plt.figure(figsize = (10,10))

x = kf.p_est[:, 0]
y = kf.p_est[:, 1]
plt.title('x(y)')
plt.plot(x, y)
plt.grid()

