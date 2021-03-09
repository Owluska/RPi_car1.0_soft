from LIBRARY.car_es_ekf import ekf
from LIBRARY.rotations import Quaternion
import numpy as np              
import pandas as pd
import matplotlib.pyplot as plt
from math import *
model = pd.read_excel('imu_model.xlsx', engine='openpyxl')

D2R = pi/180

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

wss = np.ones_like(ws) * 0.7

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



biases, stds = measure_bias_n_std(model)
#for debug purpose instead of gnss give ground truth data
var_pos = 0.001


ps_t0 = np.array([m_ps[0]])
vs_t0 = np.array([m_vs[0]])
q_t0 = Quaternion(*ws[0]).normalize().to_numpy()
p_cov_t0 = np.zeros(9)



kf = ekf()
kf.ROT =  np.eye(3) #Quaternion(*ws[0])
kf.var_imu_f = np.std(stds[:3])
kf.var_imu_w = np.std(stds[:, 3:6])
kf.var_imu_mag = np.std(stds[:, 6:9])


#because we already extract first element for initialization
# fs = fs[0:]
# ws = ws[0:]
# m_ps = m_ps[0:]
# m_ps = dts[0:]
dl =  fs.shape[0] - 1


kf.init_data(ps_t0, vs_t0, q_t0, p_cov_t0)
i = 0
l = dl
np.set_printoptions(precision=3)
#data = []
useKF = True
while(1):
    if i > l:
        break
    try:            
        print(kf.p_est[i])
        kf.loop(fs[i], ws[i], dts[i], i,
                update = useKF, sensor_var = var_pos, sensor_data = m_ps[i])
        i += 1
    except KeyboardInterrupt:
        break


mxs = m_ps[:, 0]
mys = m_ps[:, 1]

xs = kf.p_est[:, 0]
ys = kf.p_est[:, 1]

plt.plot(xs[:l], ys[:l])
plt.plot(mxs[:l],mys[:l])
plt.plot()
plt.legend(['sim', 'model'])
plt.grid()

# vs = np.array(vs)