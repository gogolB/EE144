import numpy as np

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

from math import cos, acos, sin, tan, pi, sqrt
from modern_robotics import *

#3.48
def exercise_3_48(T, S, theta):
    #print T matrix
    print("T matrix:")
    print(T)
    print("") 
    
    #define R and p
    R = [[T[0][0], T[0][1], T[0][2]],
         [T[1][0], T[1][1], T[1][2]],
         [T[2][0], T[2][1], T[2][2]]]
    print("R matrix:")
    print(R)
    print("")    
    p = [T[0][3], T[1][3], T[2][3]]
    print("p vector:")
    print(p)
    print("") 
    
    #define q:3x1, s_hat:3x1, and h: scalar
    q = S[0] #S[0] should be a 3x1 matrix
    s_hat = S[1] #S[1] should be a 3x1 matrix
    h = S[2] #S[2] should be a scalar
    
    #get normalized screw axis
    S_screw_axis = VecTose3(ScrewToAxis(q,s_hat,h)) #4x4 vector
    
    S_times_theta = theta*S_screw_axis
    S_times_theta_over2 = theta*S_screw_axis/2
    S_times_theta_over4 = theta*S_screw_axis/4
    S_times_3theta_over4 = 3*theta*S_screw_axis/4
    
    #get exponential coordinates from S*theta
    exp_S = MatrixExp6(S_times_theta)
    exp_S_theta_over2 = MatrixExp6(S_times_theta_over2)
    exp_S_theta_over4 = MatrixExp6(S_times_theta_over4)
    exp_S_3theta_over4 = MatrixExp6(S_times_3theta_over4)
    
    #calculate final result
    T_result = exp_S * T
    T_result_theta_over2 = exp_S_theta_over2 * T
    T_result_theta_over4 = exp_S_theta_over4 * T
    T_result_3theta_over4 = exp_S_3theta_over4 * T
    
    print("Resultant T for theta:")
    print(T_result)
    print("")
    
    print("Resultant T for theta/2:")
    print(T_result_theta_over2)
    print("")
    
    print("Resultant T for theta/4:")
    print(T_result_theta_over4)
    print("")
    
    print("Resultant T for 3*theta/4:")
    print(T_result_3theta_over4)
    print("")
    
    fig = plt.figure()
    ax = fig.gca(projection='3d')
    
    xs = np.zeros(5);
    ys = np.zeros(5);
    zs = np.zeros(5);
    
    xs[0] = T[0][3];
    ys[0] = T[1][3];
    zs[0] = T[2][3];
    
    xs[1] = T_result_theta_over4[0][3];
    ys[1] = T_result_theta_over4[1][3];
    zs[1] = T_result_theta_over4[2][3];
    
    xs[2] = T_result_theta_over2[0][3];
    ys[2] = T_result_theta_over2[1][3];
    zs[2] = T_result_theta_over2[2][3];
    
    xs[3] = T_result_3theta_over4[0][3];
    ys[3] = T_result_3theta_over4[1][3];
    zs[3] = T_result_3theta_over4[2][3];
    
    xs[4] = T_result[0,3];
    ys[4] = T_result[1,3];
    zs[4] = T_result[2,3];
    
    ax.plot(xs, ys, zs=zs)
    
    plt.show()