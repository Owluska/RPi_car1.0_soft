# -*- coding: utf-8 -*-
"""
Created on Mon Feb  1 15:01:38 2021

@author: User
"""
import numpy as np
# import scipy as sc


class car_iekf:
    
    g = np.array([0, 0, - 9.80655], dtype = 'double')
    a = np.zeros((1,3), dtype = 'double')
    v = np.zeros((1,3), dtype = 'double')
    p = np.zeros((1,3), dtype = 'double')
    
    ID15 = np.eye(15, dtype='double')
    ID3 = np.eye(3, dtype = 'double')
    # ID9 = np.eye(9, dtype = 'double')
    
    F = np.zeros((15, 15), dtype = 'double')
    G = np.zeros((15, 15), dtype = 'double')
    TOL = 1e-8
    #ROT = np.eye(3) + np.ones((3,3)) * TOL
    ROT = np.eye(3)
    vzero = np.zeros((1,3), dtype = 'double')
    np.seterr(divide='warn', invalid='warn')
    np.set_printoptions(precision=3)
    
    
    
    def __init__(self, gyro, acc, mag = vzero, gyro_bias = vzero, acc_bias = vzero, mag_bias = vzero,
                 gyro_std = 2e-3, acc_std = 7e-2, mag_std = 1e-3, b_gyro_std = 5e-5, b_acc_std = 4e-5, b_mag_std = 3e-5):
     
        #observation matrix
        self.H = np.zeros((6,15), dtype = 'double')
        #Q - covariance matrix of the noise propagation
        #P - state covariance matrix         
        self.P, self.Q, self.R = self.init_covariances(gyro_std, acc_std)
        self.S = self.ahat(self.H, self.P) + self.R
        #Kalman gain
        self.K = np.zeros((15,6), dtype = 'double')
        # #part of state vector: Rt, a, omega
        # self.xi = np.zeros((5,5), dtype = 'double')
        # #part of error vector: Rt, a, omega
        # self.ksi = np.zeros((1,12), dtype = 'double')
  
        # self.Wg = self.wedge(self.g.T)
        
        self.z = np.zeros((3,6), dtype = 'double')
        self.x = np.zeros((15, 3), dtype = 'double')
        #tmp = np.concatenate(gyro_bias, self.vzero, self.vzero)

        #print(gyro_bias)
        self.x[:3] = self.ROT
        self.x[9:12] = np.concatenate((gyro_bias, self.vzero, self.vzero))
        self.x[12:15] = np.concatenate((acc_bias, self.vzero, self.vzero))
        
        self.v = np.array([[.0, .0, .0]], dtype = 'double')
        self.p = np.array([[.0, .0, .0]], dtype = 'double')
        self.a = np.array([[.0, .0, .0]], dtype = 'double')
        #self.biases = np.zeros((6, 3), dtype = 'double')
    
    def ahat(self, A, H):
        return A @ H @ A.T
#       return np.einsum("ij, jk, lk->il", A, H, A)
    
    def wedge(self, phi):
        phi = phi.T
        # print(phi.shape)
        return np.array([[     0.,   -phi[2],  phi[1]],
                         [ phi[2],        0., -phi[0]],
                         [-phi[1],    phi[0],      0.]], dtype = 'double')
    
    def init_covariances(self,  gyro_std, acc_std, mag_std = 1e-3, b_gyro_std = 1e-6, b_acc_std = 14e-6, b_mag_std = 3e-5):
        Q = np.eye(15, dtype = 'double')
        
        gyro_stds = np.ones((1,3)) * gyro_std
        acc_stds = np.ones((1,3)) * acc_std
        b_gyro_stds = np.ones((1,3)) * b_gyro_std
        b_acc_stds = np.ones((1,3)) * b_acc_std
       
        Q[3:6, 3:6]     *= gyro_stds**2
        Q[6:9, 6:9]     *= acc_stds**2
        Q[9:12, 9:12]   *= b_gyro_stds**2
        Q[12:, 12:]     *= b_acc_stds**2
        
        P = np.eye(15, dtype = 'double') * 0.001
        
        R = np.eye(6, dtype = 'double')
        R[:3, :3] *= gyro_stds ** 2
        R[3:6, 3:6] *= acc_stds **2  
        return P, Q, R
    
    def normalize(self, phi):
        norm = np.linalg.norm(phi)
        if norm != 0:
            phi = phi / norm       
        return phi
    
    def SO3_exp(self, phi):
        '''phi - vector[1,3]'''
#        norm = self.normalize(phi)
        norm = np.linalg.norm(phi)
#        for e in norm:
#            if e < self.TOL:           
#                return  self.ID3 #+ self.wedge(phi)
        if norm == 0:
            #print("Zero norm")
            return self.ID3
        u = phi/norm
        
        theta = norm
        # print(u)
        c = np.cos(theta)
        s = np.sin(theta)
             
        #print(c.shape, s.shape, u.shape)
        
        #result = self.ID3 + self.wedge(u) * s + (u @ u.T-self.ID3) * (1-c)
        #print(np.cross(self.ID3, c))
        result = self.ID3 * c + self.wedge(u) * s + (u @ u.T) * (1 - c)
        # print(result.shape)
        return result
        # return phi
        
    def SO3_left_jacob(self, matrix):
        #norm = self.normalize(matrix)

#        for row in matrix:
#            for e in row:
#                if e <  self.TOL:
#                    J = self.ID3 + 1/2 * self.wedge(matrix[0])
#                    return 
        norm = np.linalg.norm(matrix)
        if(norm == 0):
            return self.ID3
        u = matrix[0]/norm
        
        theta = norm
        c = np.cos(theta)
        s = np.sin(theta)
        J = np.empty((3,3), dtype = 'double')
        # print(matrix.shape)
       
        # print((u).shape)
        J = (s/theta) * self.ID3 + (1-c)/theta * self.wedge(u)+\
        (self.ID3 - s/theta) * u @ u.T
        # print(J, end = '\n\n\n\n')
        return J
    
    def get_z(self, gyro, acc, gyro_bias = [.0,.0,.0], acc_bias = [.0,.0,.0]):
#         wedge_gyro = self.wedge(self.normalize(gyro))
#         wedge_acc = self.wedge(self.normalize(acc))
#         wedge_gyro = self.wedge(gyro)
#         wedge_acc  = self.wedge(acc)
#         self.z[:3, :3] = wedge_gyro
#         self.z[:3,3:6] = wedge_acc


         z = self.z.copy()
         z = z.T
         z[0] = gyro
         z[3] = acc 
         self.z = z.T
#         print(z.shape)
    
    #def f(self, gyro, acc, dt, gyro_bias = [.0,.0,.0], acc_bias = [.0,.0,.0]):
    def f(self, gyro, acc, dt):
        gyro_bias = self.x[9]
        acc_bias = self.x[12]
        
        self.ROT = self.ROT @ self.SO3_exp((gyro - gyro_bias)*dt)
        #self.ROT = self.ROT @ self.SO3_exp((gyro)*dt)

        #self.a = self.ROT @ (acc - acc_bias) + self.g
        self.a = self.ROT @ (acc - acc_bias) #+ self.g
        self.v += self.a * dt
        
        self.p += self.v * dt
        
        self.x[0:3] = self.ROT

        self.x[3:6]= np.concatenate((self.v, self.vzero, self.vzero))
        self.x[6:9] = np.concatenate((self.p, self.vzero, self.vzero))
        return self.x
    
#    def h(self, dt):
#        ROT = self.x[:3]
#        v = self.x[3]
#        z = self.z.copy()
#        # print(z.shape)
#        z = z.T
#        
#        z[0] = v @ ROT.T
#        z[3] = ROT.T @ (v * dt)
#        z = z.T
#        return z    
    
    def F_matrix(self, dt):
        '''F = I + A*dt
           A = dX/dX
           X = [ROT, v, p, gyro_bias, acc_bias]'''
        A = self.F.copy()
        ROT = self.x[:3]
        v_skew = self.x[3:6]
        p_skew = self.x[6:9]
        
        A[0:3, 9:12]  = -ROT
        
        A[3:6, 0:3 ]  =  self.wedge(self.g)
        A[3:6, 9:12]  = -v_skew @ ROT
        A[3:6, 12:15] = -ROT
        
        A[6:9, 3:6 ]  = self.ID3
        A[6:9, 9:12]  = -p_skew @ ROT
        # print(dt)
        self.F = self.ID15 + A * dt
        
    
    def H_matrix(self, dt):
        '''H = H*dt
           G = dX/dz
           X = [ROT, v, p, gyro_bias, acc_bias]
           Z = [imu_gyro, imu_acc]
           '''
        ROT = self.x[:3]
        
        self.H[0:3, 3:6] = ROT.T
        #self.H[0:3, 9:12] = -self.ID3
        
        self.H[3:6, 0:3] = -ROT.T @ self.wedge(self.g) 
        self.H[3:6, 12:] = -self.ID3
        self.H = self.H * dt
    
    def G_matrix(self, dt):
        '''G = [zeros + _G]*dt
           _G = dSGM/dX
           X = [ROT, v, p, gyro_bias, acc_bias]
           SGM = [sgm_gyro, sgm_acc, sgm_gyro_bias, sgm_acc_bias]
           
           _G - [12,15], to fit other matrixes dimensions it have to be [15,15],
           so we adding [0,0,0,0..0] - dim[1, 15] vector to _G
           '''
        ROT = self.x[:3]
        v_skew = self.wedge(self.v)
        p_skew = self.wedge(self.p)
        
        self.G[0:3, 3:6] = ROT
        
        self.G[3:6, 3:6] = v_skew @ ROT
        self.G[3:6, 6:9] = ROT
        
        self.G[6:9, 3:6] = p_skew @ ROT
        
        self.G[9:12, 9:12] = self.ID3
        
        self.G[12:15, 12:15] = self.ID3
        
        self.G = self.G * dt
    
    # def transponsed_Rot_from_imu(self, acc, mag):
    #     I = self.normalize(mag)
    #     K = self.normalize(acc)
    #     J = np.cross(I, K)
    #     return np.array([I, J, K])
    
    def propagate(self, gyro, acc, dt):
        ''' equations:
            F = df(x)/dx
            _x = f(x)
            _P = FPF^T + GQG^T
            '''
        self.x = self.f(gyro, acc, dt)
        #print(x)
        self.F_matrix(dt)
        self.G_matrix(dt)
        
        #self.P = self.ahat(self.F, self.P) + self.ahat(self.G, self.Q.T)
        F_inv = np.linalg.inv(self.F)
        self.P = self.F @ self.P @ F_inv + self.Q
        
    #def solve(A, B):
        
    def update(self, gyro, acc, gyro_bias, acc_bias, dt):
        ''' equations:
            H = dh(x)/dz
            y = z - h(_x)
            K = _PH^T/(H_PH^T+R)
            x = exp(Ky) * _x
            P = (I - KH)_P
            '''
        self.H_matrix(dt)
        self.get_z(gyro, acc)
        y = self.z - (self.H @ self.x).T

        #print(y, end = '\n\n\n\n')
        
        self.S = self.ahat(self.H, self.P) + self.R
        #print(self.S, end = '\n\n\n\n')
#        print(self.R, end = '\n\n\n\n')    
        S_inv = np.linalg.inv(self.S)
        #print(S_inv, end ='\n\n\n\n')
        self.K = self.P @ self.H.T @ S_inv       
        
        e = self.K @ y.T
#        e = np.zeros_like(e)
        #print(e, end = '\n\n\n\n')
        xi = np.copy(self.x)
        for i in range(3):
            i1 = i * 3
            i2 = (i+1) * 3
            xi[i1:i2] = self.SO3_left_jacob(e[i1:i2])
            self.x[i1:i2] = self.x[i1:i2] @ xi[i1:i2]

        self.x[9:15] += e[9:15]
#        self.x += e 
        self.P = (self.ID15 - self.K @ self.H) @ self.P
#        self.P  = (self.P + self.P.T)/2                  

#imu = []
#
#
#import pandas as pd
#imu_df = pd.read_excel('imu_model.xlsx', engine='openpyxl')
#
#
#cols = imu_df.columns[:5]
#cols = cols.append(imu_df.columns[8:])
#cols = cols.append(imu_df.columns[5:8])
#imu_df = imu_df[cols]
#
#len_cols = len(cols[2:])
#
## for r in imu_df.iterrows():    
##     r = r[1]['Gyrox':]
##     tmp = []
##     for l in range(len_cols):
##         tmp.append(r[l])
##     imu.append(tmp)
#
## imu = np.array([np.reshape(i, (3,3)) for i in imu])
#ts = np.array(imu_df.Time)
#gyros = np.array(imu_df[cols[2:5]])
##gyros = gyros * 180 / np.pi
#
#
#
#
#acc = np.array(imu_df[cols[5:8]])
#g = 9.780318
#acc = acc * g
#
#mags = np.array(imu_df[cols[8:11]])
#
#biases = np.zeros((1,6), dtype = 'double')
#gx, gy, gz, ax, ay, az = [],[],[],[],[],[]
#for i in range(100):
#    gx.append(gyros[i, 0])
#    gy.append(gyros[i, 1])
#    gz.append(gyros[i, 2])
#
#    ax.append(acc[i, 0])
#    ay.append(acc[i, 1])
#    az.append(acc[i, 2])
#
#biases[:,0] = np.array(gx, dtype ='double').mean()
#biases[:,1] = np.array(gy, dtype ='double').mean()
#biases[:,2] = np.array(gz, dtype ='double').mean()
#
#biases[:,3] = np.array(ax, dtype ='double').mean()
#biases[:,4] = np.array(ay, dtype ='double').mean()
#biases[:,5] = np.array(az, dtype ='double').mean()
#
#ps = []
#
#
#dt = ts[1]-ts[0]
#kf = car_iekf(gyros[0], acc[0], mags[0],
#              gyro_bias= biases[:,:3], acc_bias=biases[:,3:])
#
## kf.f(gyros[0], acc[0], dt)
## kf.get_z(gyros[0], acc[0])
#
## kf.propagate(gyros[0], acc[0], dt)
## kf.update(gyros[0], acc[0], dt)
#
#t0 = 0
#
#gyros = gyros[200:]
#acc = gyros[200:]
#mags = mags[200:]
#ts = ts[200:]
#for g, a, m, t in zip(gyros, acc, mags, ts):          
#    
#    dt = t - t0
#    if dt == 0:
#        dt = 0.01
#    kf.propagate(g, a, dt)
#    kf.update(g, a, dt)
#    # print("t: {}, p:{}".format(t, kf.p))
#    ps.append(kf.p)
#    t0 = t
#    ps.append(kf.x[6:9])      