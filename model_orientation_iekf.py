from LIBRARY.car_es_ekf import ekf
from LIBRARY.rotations import Quaternion, skew_symmetric 
import numpy as np              
import pandas as pd
import matplotlib.pyplot as plt
from math import *
model = pd.read_excel('imu_model.xlsx', engine='openpyxl')

D2R = pi/180

dl = model.shape[0] - 1 
ts = np.array([model.time]).T

dts = np.zeros_like(ts)
t0 = 0
for t, i in zip(ts, range(len(ts))):
    dt = t - t0
    #print(dt)
    t0 = t
    dts[i, :] = dt

fs = np.array([model.accx, model.accy, model.accz]).T
#gyros = np.array([model.gx, model.gy, model.gz]).T 
ws = np.array([model.gx, model.gy, model.gz]).T *  D2R

# m_as = np.array([model.rax, model.ray, model.raz]).T 
# m_gs = np.array([model.rgx, model.rgy, model.rgz]).T * D2R
m_ps = np.array([model.x, model.y, model.z]).T
m_vs = np.array([model.vx, model.vy, model.vz]).T

def measure_bias_n_std(df, points = 10):

    biases = np.empty([1, 9], dtype = 'double')
    stds = biases.copy()
    print("Measuring bias and std, amount of poinst: ", points)
        
    print()
    biases[:, :3] = np.array([np.mean(df.gx[:points]), np.mean(df.gy[points]), np.mean(df.gz[:points])])
    biases[:, 3:6] = np.array([np.mean(df.accx[:points]), np.mean(df.accy[:points]), np.mean(df.accz[:points])])
    biases[:, 6:] = np.array([np.mean(df.magx[:points]), np.mean(df.magy[:points]), np.mean(df.magz[:points])])
    
    stds[:, :3] = np.array([np.std(df.gx[:points]), np.std(df.gy[:points]), np.std(df.gz[:points])])
    stds[:, 3:6] = np.array([np.std(df.accx[:points]), np.std(df.accy[:points]), np.std(df.accz[:points])])
    stds[:, 6:9] = np.array([np.std(df.magx[:points]), np.std(df.magy[:points]), np.std(df.magz[:points])])
    
    print(stds)
    return biases, stds

# def wedge(phi):
#     phi = phi.T
#     # print(phi.shape)
#     return np.array([[     0.,   -phi[2],  phi[1]],
#                       [ phi[2],        0., -phi[0]],
#                       [-phi[1],    phi[0],      0.]], dtype = 'double')

# def SO3_exp(phi):
#     '''phi - vector[1,3]'''
#     norm = np.linalg.norm(phi)
#     if norm < 1e-8:
#         #print("Zero norm")
#         return np.eye(3)
#     u = phi/norm
    
#     c = np.cos(norm)
#     s = np.sin(norm)
         
#     result = c * np.eye(3) + s * skew_symmetric(u) + (1 - c) * (u @ u.T)
#     return result


biases, stds = measure_bias_n_std(model)
# gyro_bias= biases[:,:3]
# acc_bias=biases[:,3:]

# acc_bias = np.zeros((1,3), dtype = 'double')
# gyro_bias = np.zeros((1,3), dtype = 'double')
ps_t0 = np.array([m_ps[0]])
vs_t0 = np.array([m_vs[0]])
q_t0 = Quaternion(*ws[0]).to_numpy()
p_cov_t0 = np.zeros(9)


heading = np.array([-atan2(my, mx) for mx, my in zip(model.magx, model.magy)])
var_pos = np.std(m_ps)

kf = ekf()
kf.ROT =  np.eye(3) #Quaternion(*ws[0])
kf.var_imu_f = np.std(stds[:3])
kf.var_imu_w = np.std(stds[:, 3:6])
kf.var_imu_mag = np.std(stds[:, 6:9])

kf.init_data(ps_t0, vs_t0, q_t0, p_cov_t0)
# print(ps_t0, vs_t0)
# sdata = 'Time x y z'
# print(sdata)

toMove = True
i = 0

l = dl
#data = []
while(1):
    if i > l:
        break
    try:    

        x, y, z = kf.p_est[i,0], kf.p_est[i,1], kf.p_est[i,2] 
        sdata = "{:.4f} {:.3f} {:.3f} {:.3f}".format(ts[i, 0], x, y, z)
        #f, w, dt, k
        kf.loop(fs[i], ws[i], dts[i], i, update= True, sensor_var = var_pos, sensor_data = m_ps[i])
        #kf.update(gyros[counter], accs[counter], dts[counter])

        #print(sdata)
        i += 1
    except KeyboardInterrupt:
        break


     



# # plt.plot(ts[:counter, 0], xs)
# # plt.plot(ts[:counter, 0], ys)
# # plt.plot(ts[:counter, 0], zs)

mxs = m_ps[:, 0]
mys = m_ps[:, 1]


# plt.plot(xs, ys)
plt.plot(mxs[:l],mys[:l])
plt.plot()
# plt.legend(['model', 'sim'])
# plt.grid()

# vs = np.array(vs)