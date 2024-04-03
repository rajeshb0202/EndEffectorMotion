#!/usr/bin/env python3

from sympy import Matrix, cos, sin, Symbol, simplify, trigsimp, nsimplify, preview
from sympy.interactive import printing

# To make display prety
printing.init_printing(use_latex = True)

class GenerateHomogeneousMatrix:

    def __init__(self):
        self.theta_i = Symbol("theta_i")
        self.alpha_i = Symbol("alpha_i")
        self.r_i = Symbol("r_i")
        self.d_i = Symbol("d_i")
        self.DH_Matric_Generic = Matrix([[cos(self.theta_i), -sin(self.theta_i)*cos(self.alpha_i), sin(self.theta_i)*sin(self.alpha_i), self.r_i*cos(self.theta_i)],
                            [sin(self.theta_i), cos(self.theta_i)*cos(self.alpha_i), -cos(self.theta_i)*sin(self.alpha_i), self.r_i*sin(self.theta_i)],
                            [0, sin(self.alpha_i), cos(self.alpha_i), self.d_i],
                            [0,0,0,1]])



    def generate_matrix(self, frame_params):
        Aij = self.DH_Matric_Generic.subs(self.r_i,frame_params['r']).subs(self.alpha_i,frame_params['alpha']).subs(self.d_i,frame_params['d']).subs(self.theta_i, frame_params['theta'])
        Aij = trigsimp(Aij, method='fu')
        return Aij
    
    def generate_matrix_simplified(self, Aij_matrix):
        Aij_simplified = simplify(Aij_matrix)
        return Aij_simplified
        


# initialising the DH parameters to generate HOMOGENEOUS MATRIX
frame01_params = {'theta': Symbol("theta_1"),
                  'alpha': 1.57,
                   'r': 0.0,
                   'd': 0.0}

frame12_params = {'theta': Symbol("theta_2"),
                  'alpha': 0.0,
                  'r': Symbol("r_2"),
                  'd': 0.0}

frame13_params = {'theta': Symbol("theta_3"),
                  'alpha': 0.0,
                  'r': Symbol("r_3"),
                  'd': 0.0}


def computeMatrix():
    GHM = GenerateHomogeneousMatrix()
    A01 = GHM.generate_matrix(frame01_params)
    A12 = GHM.generate_matrix(frame12_params)
    A23 = GHM.generate_matrix(frame13_params)
    A03 = A01 * A12 * A23
    A03_simplify = GHM.generate_matrix_simplified(A03).evalf(1)
    
    # save the generated matrixes to local files
    preview(A01.evalf(1), viewer='file', filename="A0_1.png", dvioptions=['-D','300'])
    preview(A12, viewer='file', filename="A1_2.png", dvioptions=['-D','300'])
    preview(A23, viewer='file', filename="A2_3.png", dvioptions=['-D','300'])
    preview(A03.evalf(1), viewer='file', filename="A0_3.png", dvioptions=['-D','300'])
    preview(A03_simplify, viewer='file', filename="A0_3_simplified.png", dvioptions=['-D','300'])
    print("Homogeneous matrix for all frames generated")

if __name__ == "__main__":
    computeMatrix()