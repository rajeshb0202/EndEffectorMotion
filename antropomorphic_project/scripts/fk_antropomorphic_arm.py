#!/usr/bin/env python3


from generate_matrixes import GenerateHomogeneousMatrix
from sympy import Matrix, cos, sin, Symbol, simplify, trigsimp, nsimplify, preview
from sympy.interactive import printing


print("enter the values for theta1: ")
theta1 = float(input())
print("enter the values for theta2: ")
theta2 = float(input())
print("enter the values for theta3: ")
theta3 = float(input())


generate_matrix = GenerateHomogeneousMatrix()
# DH parameters for all the frames
frame01_params = {'theta': theta1,
                  'alpha': 1.57,
                   'r': 0.0,
                   'd': 0.0}

frame12_params = {'theta': theta2,
                  'alpha': 0.0,
                  'r': 1.0,
                  'd': 0.0}

frame13_params = {'theta': theta3,
                  'alpha': 0.0,
                  'r': 1.0,
                  'd': 0.0}

A01 = generate_matrix.generate_matrix(frame01_params)
A12 = generate_matrix.generate_matrix(frame12_params)
A23 = generate_matrix.generate_matrix(frame13_params)
A03 = A01 * A12 * A23


print("Position matrix")
position_matrix = A03[:3, 3]
print(position_matrix)
print("")
print("orientation matrix")
orientation_matrix = A03[:3, :3]
print(orientation_matrix)



#save the generated matrixes to local files
preview(A03, viewer='file', filename="A03_simplify_evaluated.png", dvioptions=['-D','300'])