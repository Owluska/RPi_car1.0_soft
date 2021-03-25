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
import matplotlib.pyplot as plt
from scipy.signal import butter, lfilter



#data processing
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

def butter_bandpass(lowcut, highcut, fs, order = 5):
    nyq = 0.5 * fs
    low = lowcut / nyq
    high = highcut / nyq

    b, a = butter(order, [low, high], btype = 'band')
    return b, a        

def butter_bandpass_filter(data, lowcut, highcut, fs, order = 5):
    b, a = butter_bandpass(lowcut, highcut, fs, order = order)
    y = lfilter(b,a, data)
    return y
    
# movement functions
def turn_rand():
    r = np.random.randint(low = 0, high = 3)
    if r == 0:
        return car.turn_left()
    elif r == 1:
         return  car.turn_center()
    elif r == 2:
         return  car.turn_right()
        


def check_US(USs, us, US_threshold = 15):
    labels = ['back', 'front']
    if USs.USs_out[labels[us]] == None:
        return 'us_' + labels[us] + '_none'
    elif USs.USs_out[labels[us]] < US_threshold:
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

def us_mov_script(mb, uss, uv_counter):
    template = "{}:{}:{}:{}"
    voltage_state = check_voltage(mb)
    
    if voltage_state == 'undervoltage':
        uv_counter += 1
        if uv_counter > 4:
            mvmnt_state = car.stop()
            mvmnt_state += ':' + car.turn_center()
            voltage_state = 'undervoltage'
        else:
            voltage_state = 'voltage_ok'
        
    us_front_state = check_US(uss, 0)
    
    if us_front_state.find('obstacle') != -1:
        mvmnt_state = car.move_forward()
        turn_rand()
        sleep(0.4)
        mvmnt_state += ':' + car.turn_center()
    
    us_back_state = check_US(uss, 1)   
    if us_back_state.find('obstacle') != -1:    
        mvmnt_state = car.move_backward()
        turn_rand()
        sleep(0.4)
        mvmnt_state += ':' + car.turn_center()
    
    else:
            mvmnt_state = car.move_backward()
            mvmnt_state += ':' + car.turn_center()
#                sleep(0.02)
#                self.car.stop()
#                sleep(0.01)
            uv_counter = 0
    return template.format(voltage_state, us_front_state, us_back_state, mvmnt_state)





car = rpi_movement()
car.init()


mb = mb_telemetry()
mb.init_all()

uss = US_multi()
uss.US_start()

uv_counter = 0
state = us_mov_script(mb, uss, uv_counter) 

sleep(1)


var_m = np.array([5.774, 1.119, 1.466])
var_w = np.array([0.067, 0.107, 0.029])
var_f = np.array([1.962, 3.31 , 1.603])
es_var = np.array([2.797, 0.174, 0.675])



np.set_printoptions(precision=3)
np.set_printoptions(suppress=True)

mb.telemetry()
fs,ws,ms = data_to_arrays(mb)

dt = time()
ts = np.zeros([1,1])
dts = np.zeros([1,1])



np.set_printoptions(precision=3)
np.set_printoptions(suppress=True)
useKF = True

sensors_data = np.zeros([1,9])

loop_counter = 0

sr = 1000 # sample rate in Hz
lowcut = 1
highcut = 100
while(1):
    if loop_counter > 100:
        break
    try:
        dt = time() - dt
        dts = np.append(dts, np.array([[dt]]), axis = 0)

        mb.time += dt
        ts = np.append(dts, np.array([[mb.time]]), axis = 0)
        dt = time()

#        d = np.array([uss.USs_out[l] for l in uss.USs_labels])
#        print(d)
        
        mb.telemetry()
        f,w,m = data_to_arrays(mb)

        ws = np.append(ws, w.reshape(1,3), axis = 0)
        fs = np.append(fs, f.reshape(1,3), axis = 0)
        ms = np.append(ms, m.reshape(1,3), axis = 0)
        

        try:
            state = us_mov_script(mb, uss, uv_counter)
            print(loop_counter, state)
        except Exception as e:
            template = "An exception of type {0} occured. Arguments:\n{1!r}"
            message = template.format(type(e).__name__, e.args)
            print(message)

        loop_counter += 1
        #sleep(0.1)

    except KeyboardInterrupt:
        car.turn_center()
        car.stop()
        uss.USs_stop()
        break

car.turn_center()
car.stop()
uss.USs_stop()

#_fs = butter_bandpass_filter(fs, lowcut, highcut, sr)
#_ms = butter_bandpass_filter(ms, lowcut, highcut, sr)
#_ws = butter_bandpass_filter(ws, lowcut, highcut, sr)           

plot_fig(fs, ['ACC'])
#plot_fig(_fs, ['_ACC'])
plot_fig(ws, ['GYRO'])
#plot_fig(_ws, ['_GYRO'])
plot_fig(ms, ['MAG'])