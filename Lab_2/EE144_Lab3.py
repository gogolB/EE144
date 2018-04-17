import numpy as np
from math import cos, acos, sin, tan, pi, sqrt
from modern_robotics import *

#3.43
def exercise_3_43(R):
    #check if 3x3
    if(len(R) != 3):
        print("R was not 3 elements")
        print("")
        return False
    
    elif (len(R[0]) != 3) or (len(R[1]) != 3) or (len(R[2]) != 3):
        print("One or more R[i] lists were not 3 elements")
        print("")
        return False
    
    print("R matrix: ")
    print(R)
    print("")
    
    #check if inv(R) == to R^T == I
    R_inv = RotInv(R)
    R_tran = np.array(R).T
    print("R inverse:")
    print(R_inv)
    print("R transpose:")
    print(R_tran)
    print("")
    
    if(np.array_equal(R_inv, R_tran) == False):
        print("R inverse is not equal to R transpose")
        print("")
        return False
    
    I = [[1, 0, 0], [0, 1, 0], [0, 0, 1]] #define I
    I_prime = np.matmul(R_tran, R)
    print("(R transpose * R) = I:")
    print(I_prime)
    print("")
    if (np.array_equal(I_prime, I) == False):
        print("(R transpose * R) does not equal an identity matrix")
        print("")
        return False
    
    #check if each x, y, z vector is a unit vector
    try:
        x = [R[0][0], R[1][0], R[2][0]]
        y = [R[0][1], R[1][1], R[2][1]]
        z = [R[0][2], R[1][2], R[2][2]]
        print("x: ")
        print(x)
        print("y: ")
        print(y)
        print("z: ")
        print(z)
        print("")
    except:
        print("Couldn't access an element in R")
        print("")
        return False
    
    #if we made it this far, return true
    print("R is a 3x3 rotation matrix")
    return True

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
    
def main():
    R1 = [[0, -1, 0], [1, 0, 0], [0, 0, 1]]
    #print(exercise_3_43(R1))
    
    T1 = [[1, 0, 0, 2], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]]
    S1 = [0, 0, 0]
    theta1 = pi
    exercise_3_48(T1, S1, theta1)

if __name__ == "__main__":
    main()