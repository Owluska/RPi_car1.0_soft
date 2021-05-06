# -*- coding: utf-8 -*-
"""
Created on Mon Apr 26 14:58:09 2021

@author: User
"""
import scipy.io
import numpy as np
# import matplotlib.pyplot as plt
# from matplotlib.patches import Circle
import perception_functions as pf

# Load SIFT keypoints for three images    
mat = scipy.io.loadmat('data.mat')

x1 = mat['data'][0,0]['x1']
x2 = mat['data'][0,0]['x2']
x3 = mat['data'][0,0]['x3']

# Estimate fundamental matrix
F = pf.EstimateFundametalMatrix(x1, x2)
# Load camera calibration parameters
K = mat['data'][0,0]['K'] 
# Estimate essential matrix from fundamental matrix
E = pf.EssentialMatrixFromFundamentalMatrix(F, K) 
# Load rotation matrix and camera pose from 
R2 = mat['data'][0,0]['R']
C2 = mat['data'][0,0]['C']
#set the first camera position and orientation to start of coordinate system
R1 = np.eye(3)
C1 = np.zeros((3,1))
# Obtain 3d points using correct camera pose
X = pf.LinearTriangulation(K, R1 = R1, C1 = C1, C2 = C2, R2 = R2, x1 = x1, x2 = x2)
#Find the third camera pose using Linear PnP
C3, R3 = pf.LinearPnP(X, x3, K)
#Calculate reprojection points
x1p = pf.calc_reprojection_points(K, R1, C1, X)
x2p = pf.calc_reprojection_points(K, R2, C2, X)
x3p = pf.calc_reprojection_points(K, R3, C3, X)
# Load images 
img1 = mat['data'][0,0]['img1']
img2 = mat['data'][0,0]['img2']
img3 = mat['data'][0,0]['img3']

# Display correspondence points between SIFT keypoints and reprojection
pf.DisplayDisplayCorrespondence(img1, x1, x1p)
pf.DisplayDisplayCorrespondence(img2, x2, x2p)
pf.DisplayDisplayCorrespondence(img2, x3, x3p)


# Nonlinear triangulation
_X = pf.Nonlinear_Triangulation(K, C1 = C1, R1 = R1, C2 = C2, R2 = R2, C3 = C3,
                            R3 = R3, x1 = x1, x2 = x2, x3 = x3, X0 = X)

#Calculate reprojection points
_x1p = pf.calc_reprojection_points(K, R1, C1, _X)
_x2p = pf.calc_reprojection_points(K, R2, C2, _X)
_x3p = pf.calc_reprojection_points(K, R3, C3, _X)

# Display correspondence points between SIFT keypoints and reprojection
pf.DisplayDisplayCorrespondence(img1, x1, _x1p)
pf.DisplayDisplayCorrespondence(img2, x2, _x2p)
pf.DisplayDisplayCorrespondence(img2, x3, _x3p)


import matplotlib.pyplot as plt

ax = plt.axes(projection='3d')
ax.scatter3D(X[:,0], X[:,1], X[:,2]);
plt.show()

ax = plt.axes(projection='3d')
ax.scatter3D(_X[:,0], _X[:,1], _X[:,2]);
plt.show()
