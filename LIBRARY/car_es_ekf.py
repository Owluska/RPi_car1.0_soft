import numpy as np
from LIBRARY.rotations import skew_symmetric, Quaternion


class ekf:
    def __init__(self):
        
        self.ROT = np.array([
       [ 0.99376, -0.09722,  0.05466],
       [ 0.09971,  0.99401, -0.04475],
       [-0.04998,  0.04992,  0.9975 ] ])


        self.var_imu_f = 0.01
        self.var_imu_w = 0.01
        self.var_lidar = 1.
        self.var_mag = 0.01
        

        self.g = np.array([0, 0, -9.81])       # gravity
        
        self.l_jac = np.zeros([9, 6])
        self.l_jac[3:, :] = np.eye(6)          # motion model noise jacobian

        self.h_jac = np.zeros([3, 9])
        self.h_jac[:, :3] = np.eye(3)          # measurement model jacobian
        #self.k = 0 #data counter value

        self.p_est = np.zeros([1, 3])          # position estimates
        self.v_est = np.zeros_like(self.p_est)  # velocity estimates
        self.q_est = np.zeros([1, 4])           # orientation estimates as quaternions
        self.p_cov = np.zeros([1, 9, 9])        # covariance matrices at each timestep
        self.dt = np.zeros([1,1])
        
    def init_data(self, p, v, q, p_cov):
    # Set initial values.
        self.p_est[0] = p 
        self.v_est[0] = v
        self.q_est[0] = q
        self.p_cov[0] = p_cov  # covariance of estimate


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
        p_cov = (np.eye(9) - K @ self.h_jac) @ p_cov
        return p, v, q, p_cov


    def get_F(self, dt, f, ROT):
        #(dt, C, f):
        ID = np.eye(3)
        
        F = np.eye((9), dtype = 'double')
        F[:3, 3:6] = ID * dt
        #print(f)
        F[3:6, 6:9] = -ROT @ skew_symmetric(f) * dt
        
        return F


    def get_N(self, dt):
        sigma_f =np.ones([1,3]) * self.var_imu_f
        sigma_w =np.ones([1,3]) * self.var_imu_w
        sigmas = np.append(sigma_f, sigma_w)
        #print(self.var_imu_w)
        N = np.diag(sigmas) * dt ** 2
        return N

    def propagate(self, f, w, dt, k):    
        #Linearize the motion model and compute Jacobians
        q_prev = Quaternion(*self.q_est[k])
        self.ROT = q_prev.to_mat()

        a = self.ROT @ f  + self.g
        
        p = self.p_est[k] + dt * self.v_est[k] + 0.5 * dt ** 2 * a
        v = self.v_est[k] + dt * a

        
        angle = w * dt
        qw = Quaternion(axis_angle=angle)
        q = q_prev.quat_mult_left(qw)

        F = self.get_F(dt, f, self.ROT)
        N = self.get_N(dt)
        #Propagate uncertainty 
        #print(N.shape, self.l_jac.shape)
        p_cov = F @ self.p_cov[k] @ F.T + self.l_jac @ N @ self.l_jac.T
        
        return p, v, q, p_cov
    
    def loop(self, f, w, dt, k, update = False, sensor_var = 0.0, sensor_data = 0.0):
        #print(k)
        p, v, q, p_cov = self.propagate(f, w, dt, k)
        if update:
            p, v, q, p_cov = self.measurement_update(sensor_var, sensor_data, p, v, q, p_cov)
            
        #print(p, end = '\n\n')
        
        self.p_est = np.append(self.p_est, p.reshape(1,3), axis = 0)
        self.v_est = np.append(self.v_est, v.reshape(1,3), axis = 0)
        self.q_est = np.append(self.q_est, q.reshape(1,4), axis = 0)
        self.p_cov = np.append(self.p_cov, p_cov.reshape(1, 9, 9), axis = 0)

            
            
        
        
