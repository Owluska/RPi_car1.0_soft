# Starter code for the Coursera SDC Course 2 final project.
#
# Author: Trevor Ablett and Jonathan Kelly
# University of Toronto Institute for Aerospace Studies
import pickle
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from rotations import angle_normalize, rpy_jacobian_axis_angle, skew_symmetric, Quaternion


class ekf:
    def __init__(self):
        
        self.ROT = np.array([
       [ 0.99376, -0.09722,  0.05466],
       [ 0.09971,  0.99401, -0.04475],
       [-0.04998,  0.04992,  0.9975 ] ])


        self.var_imu_f = 0.01
        self.var_imu_w = 0.01
        self.var_lidar = 1.
        

        self.g = np.array([0, 0, -9.81])       # gravity
        
        self.l_jac = np.zeros([9, 6])
        self.l_jac[3:, :] = np.eye(6)          # motion model noise jacobian

        self.h_jac = np.zeros([3, 9])
        self.h_jac[:, :3] = np.eye(3)          # measurement model jacobian
        #self.k = 0 #data counter value

        self.p_est = np.zeros([1, 3])          # position estimates
        self.v_est = np.zeros_like(self.pest)  # velocity estimates
        self.q_est = np.zeros([1, 4])           # orientation estimates as quaternions
        self.p_cov = np.zeros([1, 9, 9])        # covariance matrices at each timestep
        self.dt = np.zeros([1,1])
        
    def init_data(self, p, v, q, p_cov):
    # Set initial values.
        self.p_est = p 
        self.v_est = v
        self.q_est = q
        self.p_cov = p_cov  # covariance of estimate


    def measurement_update(self, sensor_var, y, p, v, q, p_cov):
        #Compute Kalman Gain
        I = np.identity(3)
        R = I * sensor_var
        S = self.h_jac @ p_cov @ self.h_jac.T + R
        S_inv = np.linalg.pinv(S)
        K = p_cov @ self.h_jac.T @ S_inv
        # Compute error state
        err = K @ (y - p)
        dp = err[0:3]
        dv = err[3:6]
        d_phi = err[6:9]
        # 3.3 Correct predicted state
        p += dp
        v += dv        
        q = Quaternion(axis_angle=d_phi).quat_mult_right(q)
     
        #Compute corrected covariance
        p_cov = (np.eye(9) - K @ self.h_jac) @ self.p_cov
        #save data
        self.p = np.append(self.p, p, axis = 0)
        self.v = np.append(self.v, v, axis = 0)
        self.q = np.append(self.q, q, axis = 0)
        self.p_cov = np.append(self.p_cov, p_cov, axis = 0)
       # return p_hat, v_hat, q_hat, p_cov_hat


    def get_F(self, dt, f, ROT):
        #(dt, C, f):
        ID = np.eye(3)
        
        F = np.eye((9), dtype = 'double')
        F[:3, 3:6] = ID * dt
        F[3:6, 6:9] = -ROT @ skew_symmetric(f) * dt
        
        return F


    def get_N(self, dt):
        sigma_f =np.ones([1,3]) * self.var_imu_f
        sigma_w =np.ones([1,3]) * self.var_imu_w
        sigmas = np.concatenate((sigma_f, sigma_w))
        N = np.diag(sigmas) * dt ** 2
        return N

    def propagate(self, f, w, dt, k):    
        #Linearize the motion model and compute Jacobians
        q_prev = Quaternion(*self.q[k])
        self.ROT = q_prev.to_mat()

        a = (self.ROT @ f + self.g)
        
        p = self.p_est[k] + dt * self.v_est[k] + 0.5 * dt ** 2 * a
        v = self.v_est[k] + dt * a
        
        angle = w * dt
        q = Quaternion(axis_angle=angle)
        q = q_prev.quat_mult_left(q)
        
        F = self.get_F(dt, self.ROT, f)
        N = self.get_N(dt, self.var_imu_f, self.var_imu_w)
        #Propagate uncertainty 
        p_cov = F @ self.p_cov[k] @ F.T + self.l_jac @ N @ self.l_jac.T
        
        return p, v, q, p_cov
        
        
