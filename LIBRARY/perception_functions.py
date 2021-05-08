# -*- coding: utf-8 -*-
"""
Created on Wed Apr 28 13:53:26 2021

@author: User
"""

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Circle, Arrow
from math import sqrt

def EstimateFundametalMatrix(x1, x2):
    '''
    Estimate the fundamental matrix from two image point correspondences 

    Parameters
    ----------
    x1 : size (N x 2) matrix
         of points in image 1.
    x2 : size (N x 2) matrix
         of points in image 2, each row corresponding.

    Returns
    -------
    F - size (3 x 3) fundamental matrix with rank 2.

    '''
    n = x1.shape[0]
    
    ones = np.ones((n,1))
    
    x1h = np.hstack((x1, ones))
    x2h = np.hstack((x2, ones))
    
    it = range(n)
    A = np.zeros((n, 9))
    for r1, r2, i in zip(x1h, x2h, it):
        u1 = r1[0]
        v1 = r1[1]
        
        u2 = r2[0]
        v2 = r2[1]
        A[i] = [u1*u2, u1*v2, u1, v1*u2, v1*v2, v1, u2, v2, 1]
    
    
    U, S, VT = np.linalg.svd(A, full_matrices=True)
    F = VT[:,8].reshape(3,3)
    
    uf, sf, vtf = np.linalg.svd(F)
    
    
    sf[-1] = 0
#   Reconstruction based on full SVD, 2D case:
    F = np.dot(uf[:, :3] * sf, vtf)
    return F


def EssentialMatrixFromFundamentalMatrix(F,K):
    '''
    Use the camera calibration matrix to esimate the Essential matrix
    Inputs:
    K - size (3 x 3) camera calibration (intrinsics) matrix%     F - size (3 x 3) fundamental matrix from EstimateFundamentalMatrix
    Outputs:
    E - size (3 x 3) Essential matrix with singular values (1,1,0)

    '''
    E = K.T @ F @ K
    U, _, Vt = np.linalg.svd(E)
    I = np.eye(3)
    
    E = U @ I @ Vt.T
    return E

def skew(v):
    skew_sym = np.array([[   0, -v[2],  v[1]],
                         [ v[2],    0, -v[0]],
                         [-v[1], v[0],     0]])
    return skew_sym

def LinearTriangulation(K, C1, R1, C2, R2, x1, x2):
    '''
    Find 3D positions of the point correspondences using the relative
    position of one camera from another

    Parameters
    ----------
    K : size (3 x 1)
        intrinsic camera parameters.
    C1 : size (3 x 1)
         translation of the first camera pose.
    R1 : size (3 x 3)
         rotation of the first camera pose.
    C2 : size (3 x 1)
         translation of the second camera.
    R2 : size (3 x 3)
         rotation of the second camera pose.
    x1 : size (N x 2)
         matrix of points in image 1.
    x2 : size (N x 2) matrix
         of points in image 2, each row corresponding to x1.

    Returns
    -------
    X : size (N x 3) matrix
        whos rows represent the 3D triangulated points.

    '''
    n, m = x1.shape
    # print(n, m)
    
    #A = np.zeros((4, 6))
    X = np.zeros((n, 3))
    
    ones = np.ones((n,3))
    
    x1h = np.hstack((x1, ones))
    x2h = np.hstack((x2, ones))
    
    
    t1 = np.zeros((3,4))
    t2 = np.zeros((3,4))
    
    it = range(n)
    
    for p1, p2, i in zip(x1h, x2h, it):               
        sk1 = skew(p1)
        sk2 = skew(p2)
        
        t1[:, :3] = R1
        t1[:, 3] = (-R1 @ C1).T        
        P1 = K @ t1
        
        t2[:, :3] = R2
        t2[:, 3] = (-R2 @ C2).T  
        P2 = K @ t2
        A = np.vstack((sk1 @ P1, sk2 @ P2))

        U, S, VT = np.linalg.svd(A)
        V = VT.T

        X_b = V[0:3, -1]/V[-1,-1]

        X[i] = X_b.T
    
    return X

def LinearPnP(X, x3, K):
    '''
     Getting pose from 2D-3D correspondences
     Inputs:
         X - size (N x 3) matrix of 3D points
         x - size (N x 2) matrix of 2D points whose rows correspond with X
         K - size (3 x 3) camera calibration (intrinsics) matrix
     Outputs:
         C - size (3 x 1) pose transation
         R - size (3 x 1) pose rotation
    
     IMPORTANT NOTE: While theoretically you can use the x directly when solving
     for the P = [R t] matrix then use the K matrix to correct the error, this is
     more numeically unstable, and thus it is better to calibrate the x values
     before the computation of P then extract R and t directly
       
    '''
    n = x3.shape[0]
    # A = np.zeros((3*n,12))
    
    
    # onesv = np.ones((1,4))
    # a = np.zeros((3,12))
    # a[0, 0:4] = onesv
    # a[1, 4:8] = onesv
    # a[2, 8:12] = onesv
    
    # #print(a)
    # ones = np.ones((n,1))
    # Xh = np.hstack((X, ones))
    # x3h = np.hstack((x3, ones))
    # it = range(n)
    # for xs, xs3, i in zip(Xh, x3h, it):
    #     xskew = skew(xs3)     
    #     A[i:i+3] = xskew @ a
    ones = np.ones((n,1))
    Xh = np.hstack((X, ones))
    x3h = np.hstack((x3, ones))   
    A = np.zeros((2*n,12))
    it = range(n)
    for Xs, xs3, i in zip(Xh, x3h, it):   
        A[2*i, 4:8] = -1 * Xs
        A[2*i, 8:12] = xs3[1] * Xs
        
        A[2*i+1, 0:4] = Xs
        A[2*i+1, 8:12] = -xs3[0] * Xs


        

    _, __, VTa = np.linalg.svd(A) 
    Va = VTa.T
    p = (Va[:, -1]/ Va[-1,-1])

    p = p.reshape((3,4))
    
    P = np.linalg.pinv(K) @ p
     
    Rp = P[0:3, 0:3]
    

    Ur, Sr, VTr = np.linalg.svd(Rp) 
    
    det = np.linalg.det(Ur @ VTr)
    
    #print(VTr, det)
    if det > 0:
        R3 = Ur @ VTr
        t = P[:, -1] / Sr[0]
    else:
        R3 = -Ur @ VTr
        t = -P[:, -1] / Sr[0]        
    
    #print(t)
    C3 = -R3.T @ t
    C3 = C3.reshape(3,1)
    return C3, R3

def DisplayDisplayCorrespondence(image, projection_points, reprojection_points):
    fig, ax = plt.subplots(1)
    ax.imshow(image)
    height, width = image.shape[:2]
    # print()
    prj_points = [Circle((p[0], p[1]), radius=3, color='red') for p in projection_points]
    #reprj_points = [Circle((rp[0], rp[1]), radius=2, color='blue') for rp in reprojection_points]
    reprj_points = [Arrow(p[0], p[1], rp[0] - p[0], rp[1] - p[1], color='blue')\
                    for rp, p in zip(reprojection_points, projection_points)]

    #max_errs = [max(errors[:, 0]), max(errors[:, 1])]
    for prj, reprj in zip(prj_points, reprj_points):
        ax.add_patch(prj)
        ax.add_patch(reprj)
    
    err = 0
    for rp, p in zip(reprojection_points, projection_points):    
        err += (p-rp[:2]) ** 2
    err = sqrt(sum(err)/projection_points.shape[0])
    
    text = "Error: {:.2f}\n".format(err)
    plt.text(height, 0 + 10, text, ha="center", family='sans-serif', size=10)
    plt.show(fig)
    
def calc_reprojection_points(K, R, C, X):
    n = X.shape[0]
    xrp = K @ R @ (X.T - np.tile(C, (1, n)))
    xrp = xrp /np.tile(xrp[2, :], (3, 1))
    xrp = xrp.T
    return xrp


def Nonlinear_Triangulation(K, C1, R1, C2, R2, C3, R3, x1, x2, x3, X0):
    '''
    Nonlinear_Triangulation
    Refining the poses of the cameras to get a better estimate of the points
    3D position
    Inputs: 
     K - size (3 x 3) camera calibration (intrinsics) matrix
     x
    Outputs: 
     X - size (N x 3) matrix of refined point 3D locations     
    '''
    n = X0.shape[0]
    it = range(n)
    X = np.copy(X0)
    #print(X[0].shape)
    for xs1, xs2, xs3, Xs0, i in zip(x1, x2, x3, X0, it):
        X[i] = Single_Point_Nonlinear_Triangulation(K, C1, R1, C2, R2, C3, R3, xs1, xs2, xs3, Xs0)[:,0]
        
    return X

def Single_Point_Nonlinear_Triangulation(K, C1, R1, C2, R2, C3, R3, x1, x2, x3, X0):
    '''
    

    Parameters
    ----------
    K : size (3 x 3)
        intrinsic camera parameters.
    C1 : size (3 x 1)
        camera-1 position (shift).
    R1 : size (3 x 3)
        camera-1 rotation.
    C2 : size (3 x 1)
        camera-2 position (shift).
    R2 : size (3 x 3)
        camera-2 rotation.
    C3 : size (3 x 1)
        camera-3 position (shift).
    R3 : size (3 x 3)
        camera-3 rotation.
    x1 : size (1 x 2)
        projection poin of camera-1.
    x2 : size (1 x 2)
        projection poin of camera-1.
    x3 : size (1 x 2)
        projection poin of camera-1.
    X0 :size (1 x 3)
        point 3D location.

    Returns
    -------
    X :size (1 x 3)
       triangulated point 3D location..
    
    https://www.coursera.org/learn/robotics-perception/programming/cUt6t/structure-from-motion 
                                                      -> RoboticsPerceptionWeek4Assignment.pdf
    '''
    J = np.zeros((6,3))
    J[0:2, :] = Jacobian_Triangulation(C1, R1, K, X0)
    J[2:4, :] = Jacobian_Triangulation(C2, R2, K, X0)
    J[4:6, :] = Jacobian_Triangulation(C3, R3, K, X0)
    

   
    # equation (4)
    X0 = X0.reshape((3,1))
    #print(X0.shape, C3.shape)
    uvw1 = K @ R1 @ (X0 - C1)
    uvw2 = K @ R2 @ (X0 - C2)
    uvw3 = K @ R3 @ (X0 - C3) 
    u1,v1,w1 = uvw1[0], uvw1[1], uvw1[2]  
    u2,v2,w2 = uvw2[0], uvw2[1], uvw2[2]   
    u3,v3,w3 = uvw3[0], uvw3[1], uvw3[2]
    
    # equation (6)
    b = np.hstack((x1,x2,x3))
    b = b.reshape((6,1))
    # equation (7)
    #print(u1.shape)
    f = np.array([u1/w1, v1/w1, u2/w2, v2/w2, u3/w3, v3/w3])
    # equation (5)
    
    JM = np.linalg.inv(J.T @ J)
    #print(b.shape,  f.shape)
    dX = JM @ J.T @ (b - f)
    X = X0 + dX
    #print(X.shape)
    return X
    
def Jacobian_Triangulation(C, R, K, X):
    '''
    Parameters
    ----------
    C : size (3 x 1)
        camera position (shift).
    R : size (3 x 3)
        camera rotation.
    K : size (3 x 3)
        intrinsic camera parameters.
    X : size (n x 3)
        points 3D locations.

    Returns
    -------
    J : size (2 x 1)
        Jacobian matrix of nonlinear triangulation least-sq errs.
        
    https://www.coursera.org/learn/robotics-perception/programming/cUt6t/structure-from-motion 
                                                      -> RoboticsPerceptionWeek4Assignment.pdf
    '''
    # equation (4)
    C = C.reshape(3)
    uvw = K @ R @ (X - C)
    
    u = uvw[0]
    v = uvw[1]
    w = uvw[2]
    
    fx = K[0,0]
    fy = K[1,1]
    
    px = K[0,2]
    py = K[1,2]
    
    # equations (10-23)
    du = np.array([fx * R[0,0] + px * R[2,0], fx * R[0,1] + px * R[2,1], fx * R[0,2] + px *R[2,2]])
    dv = np.array([fy * R[1,0] + py * R[2,0], fy * R[1,1] + py * R[2,1], fy * R[1,2] + py *R[2,2]])
    dw = R[2, :]
    # equations (8-9)
    J = np.array([(w * du - u * dw)/w ** 2, (w * dv - v * dw)/w ** 2])

    #print()
    return J
    
    
    