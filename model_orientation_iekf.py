from LIBRARY.car_iekf_2 import car_iekf 
import numpy as np              
import pandas as pd
import matplotlib.pyplot as plt
import math
model = pd.read_excel('imu_model.xlsx', engine='openpyxl')

D2R = math.pi/180

dl = model.shape[0] - 1 
ts = np.array([model.time]).T

dts = np.zeros_like(ts)
t0 = 0
for t, i in zip(ts, range(len(ts))):
    dt = t - t0
    #print(dt)
    t0 = t
    dts[i, :] = dt

accs = np.array([model.accx, model.accy, model.accz]).T
#gyros = np.array([model.gx, model.gy, model.gz]).T 
gyros = np.array([model.gx, model.gy, model.gz]).T * D2R

m_ps = np.array([model.x, model.y, model.z]).T
m_vs = np.array([model.vx, model.vy, model.vz]).T

def measure_bias_n_std(df, points = 10):

    biases = np.empty((1, 6), dtype = 'double')
    stds = biases.copy()
    print("Measuring bias and std, amount of poinst: ", points)
        
    print()
    biases[:, :3] = np.array([np.mean(df.gx[:points]), np.mean(df.gy[points]), np.mean(df.gz[:points])])
    biases[:, 3:] = np.array([np.mean(df.accx[:points]), np.mean(df.accy[:points]), np.mean(df.accz[:points])])
    
    stds[:, :3] = np.array([np.std(df.gx[:points]), np.std(df.gy[:points]), np.std(df.gz[:points])])
    stds[:, 3:] = np.array([np.std(df.accx[:points]), np.std(df.accy[:points]), np.std(df.accz[:points])])
    return biases, stds

ps, vs, acs = [], [], []
e1, e2, zzs = [],[],[]
ys = []
biases, stds = measure_bias_n_std(model, points= dl)
# gyro_bias= biases[:,:3]
# acc_bias=biases[:,3:]

# acc_bias = np.zeros((1,3), dtype = 'double')
# gyro_bias = np.zeros((1,3), dtype = 'double')
ps_t0 = np.array([m_ps[0]])
vs_t0 = np.array([m_vs[0]])
kf = car_iekf(gyros, accs, gyro_std = stds[:, :3], acc_std = stds[:, 3:], v = vs_t0,  p = ps_t0) 


# print(ps_t0, vs_t0)
# sdata = 'Time x y z'
# print(sdata)

toMove = True
counter = 0


#data = []
while(1):
    if counter > dl:
        break
    try:    

        x, y, z = kf.p[0][0], kf.p[0][1], kf.p[0][2]
        vx, vy, vz = kf.v[0][0], kf.v[0][1], kf.v[0][2]
        ax, ay, az = kf.v[0][0], kf.v[0][1], kf.v[0][2]
        
        ps.append([x,y,z])
        vs.append([vx,vy,vz])
        acs.append([ax,ay,az])
        
        e1.append(kf.e[3])
        e2.append(kf.e[6])
        ys.append(np.max(kf.y))
        
        kf.propagate(gyros[counter], accs[counter], dts[counter])
        kf.update(gyros[counter], accs[counter], dts[counter])
        zzs.append(kf.z)        
        
        # vx, vy, vz = kf.v[0][0], kf.v[0][1], kf.v[0][2]

        sdata = "{:.4f} {:.3f} {:.3f} {:.3f}".format(ts[counter][0], x, y, z)
        #print(sdata)
        counter += 1
    except KeyboardInterrupt:
        break


     

xs = [ps[i][0] for i in range(counter)]
ys = [ps[i][1] for i in range(counter)]
zs = [ps[i][2] for i in range(counter)]

# plt.plot(ts[:counter, 0], xs)
# plt.plot(ts[:counter, 0], ys)
# plt.plot(ts[:counter, 0], zs)
mxs = m_ps[:, 0]
mys = m_ps[:, 1]
plt.plot(xs, ys)
plt.plot(mxs,mys)
plt.plot()
#plt.legend(['x(t)','y(t)','z(t)'])
plt.grid()