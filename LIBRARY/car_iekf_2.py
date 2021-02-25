# -*- coding: utf-8 -*-
"""
Created on Mon Feb  1 15:01:38 2021

@author: User
"""
import numpy as np
# import scipy as sc


class car_iekf:
    
    TOL = 1e-8
    np.seterr(divide='warn', invalid='warn')
    np.set_printoptions(precision=3)
    
    state_dim = 9
    
    g = np.array([0, 0, 9.80655], dtype = 'double')
    
    ID = np.eye(state_dim, dtype='double')
    ID3 = np.eye(3, dtype = 'double')
    # ID9 = np.eye(9, dtype = 'double')
    
    F = np.zeros((state_dim, state_dim), dtype = 'double')
   
    ROT = np.eye(3)
    v0 = np.zeros((1,3), dtype = 'double')
          
    def __init__(self, gyro, acc, gyro_std = 2e-3, acc_std = 7e-2, v = v0, p = v0, a = v0):
        self.H = np.zeros((6,self.state_dim), dtype = 'double')
    
        self.P, self.Q, self.R = self.init_covariances(self.state_dim, gyro_std, acc_std)
        self.S = self.ahat(self.H, self.P) + self.R

        self.K = np.zeros((self.state_dim,6), dtype = 'double')

        self.Wg = self.wedge(self.g.T)
        
        self.z = np.zeros((3,6), dtype = 'double')
        self.y = np.copy(self.z)
        self.x = np.zeros((self.state_dim, 3), dtype = 'double')
        self.e = np.zeros_like(self.x)       
        
        self.p = p      
        self.v = v
        self.a = a

        self.x[:3] = self.ROT
        self.x[3] = self.v
        self.x[6] = self.p
        

    
    def ahat(self, A, H):
        return A @ H @ A.T
#       return np.einsum("ij, jk, lk->il", A, H, A)
    
    def wedge(self, phi):
        phi = phi.T
        # print(phi.shape)
        return np.array([[     0.,   -phi[2],  phi[1]],
                         [ phi[2],        0., -phi[0]],
                         [-phi[1],    phi[0],      0.]], dtype = 'double')
    
    def init_covariances(self,  dim, gyro_std, acc_std):
        Q = np.eye(dim, dtype = 'double') * 0.005
        
        # ROT_stds = np.eye(3)* 0.01
        gyro_stds = np.ones((1,3)) * gyro_std
        acc_stds = np.ones((1,3)) * acc_std

        # Q[0:3, 0:3]   = ROT_stds
        # Q[3:6, 3:6]  *= gyro_stds ** 2
        # Q[6:9, 6:9]  *= acc_stds ** 2
        
        P = np.eye(dim, dtype = 'double') * 0.1
        
        R = np.eye(6, dtype = 'double')
        R[:3, :3] *= gyro_stds ** 2
        R[3:6, 3:6] *= acc_stds ** 2  
        return P, Q, R
    
    def normalize(self, phi):
        norm = np.linalg.norm(phi)
        if norm != 0:
            phi = phi / norm       
        return phi
    
    def SO3_exp(self, phi):
        '''phi - vector[1,3]'''
        norm = np.linalg.norm(phi)
        if norm == 0:
            #print("Zero norm")
            return self.ID3
        u = phi/norm
        
        c = np.cos(norm)
        s = np.sin(norm)
             
        result = c * self.ID3 + s * self.wedge(u) + (1 - c) * (u @ u.T)
        return result
        
    def SO3_left_jacob(self, matrix):

#        for row in matrix:
#            for e in row:
#                if e <  self.TOL:
#                    J = self.ID3 + 1/2 * self.wedge(matrix[0])
#                    return 
        norm = np.linalg.norm(matrix)
        if(norm == 0):
            return self.ID3
        u = matrix[0]/norm
        
        c = np.cos(norm)
        s = np.sin(norm)
        J = np.empty((3,3), dtype = 'double')
        # print(matrix.shape)
       
        # print((u).shape)
        J = (s/norm) * self.ID3 + (1-c)/norm * self.wedge(u)+\
        (self.ID3 - s/norm) * u @ u.T
        # print(J, end = '\n\n\n\n')
        return J
    
    def h(self, gyro, acc):
        self.z[:, 0] = gyro
        self.z[:, 3] = acc
         
         # self.z[:3, 0:3] = self.wedge(gyro)
         # self.z[:3, 3:6] = self.wedge(acc)


    
    def f(self, gyro, acc, dt):

        
        self.a = self.ROT @ acc #+ self.g                      
        
        self.v += self.a * dt
        self.p += self.v * dt + 1/2 * self.a * dt ** 2
               
        self.ROT = self.ROT @ self.SO3_exp(gyro*dt)
        #self.ROT = self.ROT @ self.wedge(gyro) * dt      
        
        self.x[0:3] = self.ROT
        self.x[3]= self.v
        self.x[6] = self.p
        
    def F_matrix(self, dt):
        '''F = I + A*dt
           A = dX/dX
           X = [ROT, v, p, gyro_bias, acc_bias]'''
        A = self.F.copy()
           
        A[3:6, 0:3]  =  self.Wg
        A[6:9, 3:6]  =  self.ID3

        # print(dt)
        self.F = self.ID + A * dt
        
    
    def H_matrix(self):
        '''H = H*dt
           G = dX/dz
           X = [ROT, v, p, gyro_bias, acc_bias]
           Z = [imu_gyro, imu_acc]
           '''
      
        self.H[0:3, 3:6] = self.ROT.T        
        self.H[3:6, 0:3] = self.ROT.T @ self.g #+ self.g

    
    
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
        self.f(gyro, acc, dt)
        #print(x)
        self.F_matrix(dt)
        #F_inv = np.linalg.inv(self.F)
        self.P = self.ahat(self.F, self.P) + self.Q
        
    #def solve(A, B):
        
    def update(self, gyro, acc, dt):
        ''' equations:
            H = dh(x)/dz
            y = z - h(_x)
            K = _PH^T/(H_PH^T+R)
            x = exp(Ky) * _x
            P = (I - KH)_P
            '''
        self.H_matrix()
        self.h(gyro, acc)
        #print((self.H @ self.x).T)
        self.y = self.z - (self.H @ self.x).T
        #print(np.max(y), end = '\n\n\n\n')
        
        self.S = self.ahat(self.H, self.P) + self.R
        #print(self.S, end = '\n\n\n\n')
#        print(self.R, end = '\n\n\n\n')    
        S_inv = np.linalg.inv(self.S)
        #print(S_inv, end ='\n\n\n\n')
        self.K = self.P @ self.H.T @ S_inv       
        
        self.e = self.K @ self.y.T
        # print(e, end = '\n\n\n\n')
#        e = np.zeros_like(e)
        #print(e, end = '\n\n\n\n')

        # self.x[0:3] = self.x[0:3] @ self.SO3_left_jacob(self.e[0:3])
        # self.x[3] = self.x[3] @  self.SO3_left_jacob(self.e[3:6])
        # self.x[6] = self.x[6] @  self.SO3_left_jacob(self.e[6:9])
        
        self.x += self.e         
        self.ROT = self.x[0:3]
        self.v[:] = self.x[3]
        self.p[:] = self.x[6]
        
        # print(self.p.shape)

        self.P = (self.ID - self.K @ self.H) @ self.P
        #self.P  = (self.P + self.P.T)/2                  

   