#!/usr/bin/env python3

import numpy as np

# elbow_config = "all" or "plus-minus"

class ComputeInverseKinematics:
    def __init__(self):
        self.threshold = 0.01

    def check_solutions_possible(self, theta_1, theta_2, theta_3, x, y, z):
    # this method checks whether the computed theta values are satisfying the equations or not
        eq1 = (np.cos(theta_1)) * (np.cos(theta_2)  +  np.cos(theta_2 + theta_3)) - x
        eq2 = (np.sin(theta_1)) * (np.cos(theta_2)  +  np.cos(theta_2 + theta_3)) - y
        eq3 = np.sin(theta_2) + np.sin(theta_2 + theta_3) - z
        if (np.abs(eq1) < self.threshold and np.abs(eq2) < self.threshold and np.abs(eq3) < self.threshold):
            return True
        return False
    

    def compute_theta2(self, theta_1, theta_3, y, z):
    # this method computes the theta_2 values for the given theta_1, theta_3, y, z values
        try:
            theta_2_solutions = [np.arctan2(z*np.sin(theta_1), y) -(theta_3/2), np.arctan2(z*np.sin(theta_1), y) - (theta_3/2) + np.pi, np.arctan2(z*np.sin(theta_1), y) - (theta_3/2) - np.pi]
        except Exception as e:
            print("error in computing for theta_2: ", e)
        return theta_2_solutions
    

    def compute_theta1(self, x, y):
    # this method computes the theta_1 values for the given x, y values
        theta_1_solutions = [np.arctan2(y, x), np.arctan2(y, x) - np.pi, np.arctan2(y, x) + np.pi]
        return theta_1_solutions
    

    def compute_theta3(self, x, y, z):
    # this method computes the theta_3 values for the given x, y, z values
        d = np.sqrt(x**2 + y**2 + z**2)
        acos_arg = (d**2 - 2) / 2
        if (acos_arg > 1 or acos_arg < -1):
            print("the end effector cannot reach to the given values of x, y, z... please try again with different values..")
            exit(1)
        theta_3_solutions = [np.arccos(acos_arg), -np.arccos(acos_arg)]
        return theta_3_solutions



    def calculate_ik(self, Pee_x, Pee_y, Pee_z, elbow_config):
    # this method computes the possible theta values for all the joints.
        theta_1_solutions = self.compute_theta1(Pee_x, Pee_y)
        theta_3_solutions = self.compute_theta3(Pee_x, Pee_y, Pee_z)
        all_joints_solutions = []
        plus_minus_solutions = []
        for theta_1 in theta_1_solutions:
            if (theta_1 < -np.pi or theta_1 > np.pi):
                continue

            for theta_3 in theta_3_solutions:
                if (theta_3 < -np.pi or theta_3 > np.pi):
                    continue

                # compute theta_2 solutions
                solutions = self.compute_theta2(theta_1, theta_3, Pee_y, Pee_z)
                for theta_2 in solutions:
                    if (theta_2 <= -np.pi or theta_2 >= np.pi):
                        continue

                    # check whether computed theta values are indeed satisfying the equations or not
                    solution_possible_or_not = self.check_solutions_possible(theta_1, theta_2, theta_3, Pee_x, Pee_y, Pee_z)
                    if (not solution_possible_or_not):
                        continue

                    # check for theta values within limits
                    within_limit = False
                    if (theta_2 > -np.pi/4 and theta_2 < 3* np.pi/4 and theta_3 > -3*np.pi/4 and theta_3 < 3*np.pi/4):
                        within_limit = True
                                        
                    all_joints_solutions.append([theta_1, theta_2, theta_3, within_limit])

                    if (within_limit and theta_2 > 0 and theta_3 < 0):
                        plus_minus_solutions.append([theta_1, theta_2, theta_3, within_limit])
                        

        if (elbow_config == "all"):
            return all_joints_solutions
        elif(elbow_config == "plus-minus"):
            return plus_minus_solutions
            
                   




def enter_end_effector_position():
# this function takes the end effector position as input from the user
    while True:
        print()
        print("enter the position of the end effector in x direction: ")
        x = float(input())
        print("enter the position of the end effector in y direction: ")
        y = float(input())
        print("enter the position of the end effector in z direction: ")
        z = float(input())
        d = np.sqrt(x**2 + y**2 + z**2)
        acos_arg = (d**2 - 2) / 2
        if (acos_arg > 1 or acos_arg < -1):
            print("the end effector cannot reach to the given values of x, y, z... please try again with different values..")
            # empty line
            print()
            print()
            continue
        else:
            print()
            print()
            break
    return x, y, z 


def start_compute_IK():
# this function starts the computation of the inverse kinematics
    x,y,z = enter_end_effector_position()
    Pee = ComputeInverseKinematics()
    solutions = Pee.calculate_ik(x, y, z, "all")                        
    for solution in solutions:
        print(f"Angles thetas solved = {solution[:3]}, solution_possible = {solution[3]} ")

if __name__ == "__main__":
    start_compute_IK()




