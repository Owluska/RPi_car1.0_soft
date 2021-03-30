# -*- coding: utf-8 -*-
"""
Created on Mon Feb  1 15:01:38 2021

@author: User
"""
# import numpy as np
# # import scipy as sc
from LIBRARY.rpi_car import rpi_movement
from LIBRARY.rpi_telemetry import mb_telemetry
from LIBRARY.rpi_US_multi import US_multi

from time import time, sleep
import numpy as np
from math import sqrt, sin, cos, atan2
import matplotlib.pyplot as plt
from ahrs.filters import EKF
from scipy.spatial.transform import Rotation as R


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
        


def check_US(USs, us, US_threshold1 = 20, US_threshold2 = 50):
    labels = ['back', 'front']
    if USs.USs_out[labels[us]] == None:
        return 'us_' + labels[us] + '_none'
    elif USs.USs_out[labels[us]] < US_threshold1:
        return 'obstacle_' + labels[us]
    elif USs.USs_out[labels[us]] < US_threshold2:
        return 'target' + labels[us]
    else:
        return 'us_' + labels[us] + '_ok'

def check_voltage(mb, voltage_threshold = 7.0):
    if mb.motors_voltage == None:
        return 'voltm_none'
    elif(mb.motors_voltage < voltage_threshold):
        return 'undervoltage'
    else:
        return 'voltage_ok'

def fromUS_mov_script(mb, uss, uv_counter):
    template = "{}:{}:{}:{}"
    voltage_state = check_voltage(mb)
    us_front_state = check_US(uss, 0)
    us_back_state = check_US(uss, 1)
    
    if voltage_state == 'undervoltage':
        uv_counter += 1
        if uv_counter > 4:
            mvmnt_state = car.stop()
            mvmnt_state += ':' + car.turn_center()
            voltage_state = 'undervoltage'
        else:
            voltage_state = 'voltage_ok'
        
    if us_front_state.find('target') != -1:
        mvmnt_state = car.move_forward()
        rot_state = turn_rand()
        sleep(0.5)
        mvmnt_state += ':' + rot_state
        uv_counter = 0
    elif us_back_state.find('target') != -1:    
        mvmnt_state = car.move_backward()
        rot_state = turn_rand()
        sleep(0.5)
        mvmnt_state += ':' + rot_state
        uv_counter = 0
    else:
        mvmnt_state = car.stop()
        mvmnt_state += ':' + car.turn_center()
#                sleep(0.02)
#                self.car.stop()
#                sleep(0.01)
        uv_counter = 0
    return template.format(voltage_state, us_front_state,\
                           us_back_state, mvmnt_state)




#motors 
car = rpi_movement()
car.init()

#telemetry
mb = mb_telemetry()
mb.init_all()

uss = US_multi()
uss.US_start()

#starts movemtnt before kalman init
uv_counter = 0
state = fromUS_mov_script(mb, uss, uv_counter) 

sleep(2)

#sensors variances
stds_m = np.array([5.774, 1.119, 1.466])
stds_w = np.array([0.067, 0.107, 0.029])
stds_f = np.array([1.962, 3.31 , 1.603])
es_stds = np.array([2.797, 0.174, 0.675])


std_f = np.max(stds_f)
std_w = np.max(stds_w)
std_m = np.max(stds_m)
#init data
mb.telemetry()
fs,ws,ms = data_to_arrays(mb)
_fs = np.copy(fs)
_ws = np.copy(ws)
#fs -= bias_f
#ws -= bias_w
#ms -= bias_m
t = time()
ts = np.zeros([1,1])
dts = np.zeros([1,1])

#create n init kalman
ekf = EKF(gyr = ws, acc = fs, mag = ms, noises = [std_w, std_f, std_m])

#set print options
np.set_printoptions(precision=3)
np.set_printoptions(suppress=True)

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

ps = np.zeros((1,3))
vs = np.zeros((1,3))
accs = np.zeros((1,3))

q = np.array([[1,0,0,0]])
g = np.array([[0,0,-9.8]])
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
        
        ts = np.append(dts, np.array([[mb.time]]), axis = 0)
        dts = np.append(dts, np.array([[dt]]), axis = 0)
        ws = np.append(ws, w.reshape(1,3), axis = 0)
        fs = np.append(fs, f.reshape(1,3), axis = 0)
        ms = np.append(ms, m.reshape(1,3), axis = 0)
        
        _fs = average_measurments(samples = samples, counter_value = counter, raw = fs, averaged = _fs, atype=av_type)
        _ws = average_measurments(samples = samples, counter_value = counter, raw = ws, averaged = _ws, atype=av_type)
        

        print(templ.format(dt, *ps[k]), end = '')
        if counter % samples == 0:
            _f = average_measurment(samples = samples, counter_value = counter, raw = fs, atype=av_type)
            _w = average_measurment(samples = samples, counter_value = counter, raw = ws, atype=av_type)
            
            _q = ekf.update(q[k], gyr = _ws[k], acc = _fs[k], mag = ms[k])
            q = np.append(q, _q.reshape(1,4), axis = 0 )
#            ROT = R.from_quat(_q).as_matrix()
#            a  = ROT @ _f - g
            a = ekf.acc - g
            p = ps[k] + vs[k] * dt + 1/2 * a * dt ** 2
            v = vs[k] + a * dt
            
            k += 1


        
        ps = np.append(ps, p.reshape(1,3), axis = 0)
        vs = np.append(vs, v.reshape(1,3), axis = 0)
        accs = np.append(accs, a.reshape(1,3), axis = 0)
        if toMove:
            try:
                state = fromUS_mov_script(mb, uss, uv_counter)
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
plot_fig(ps, 'pos')
plot_fig(vs, 'vel')
plot_fig(accs, 'acc')

fig = plt.figure(figsize = (10,10))

x = ps[:, 0]
y = ps[:, 1]
plt.title('x(y)')
plt.plot(x, y)
plt.grid()

