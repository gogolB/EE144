#!/usr/bin/env python3
# -*- coding: utf-8 -*-

'''from modern_robotics import *'''
import numpy as np
"""
Created on Tue Apr 17 13:41:30 2018

@author: ee144
"""

'''write a function that returns "true" if a given 3x3 matrix a given 3x3
matrix is within epislon of an element of so(3) and "false" otherwise'''

''' so(3) is the set of all 3x3 real skew-symmetric matricies'''

'''x_Transpose = np.transpose(x).T'''

x = np.array([[0, -1, 2],[1, 0, -3],[-2, 3, 0]])
epsilon = 1

def test(x,eps):
    x_t = np.transpose(x)
    constraint = np.absolute(x + x_t)
    print(constraint)
    if ((constraint < eps).all()):
        print("true")
    else:
        print("false")
    return()
    
test(x,epsilon)