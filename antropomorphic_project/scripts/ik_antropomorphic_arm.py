#!/usr/bin/env python3

import numpy as np

threshold = 0.01

def enter_end_effector_position():
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
            print()
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



def check_solutions_possible(theta_1, theta_2, theta_3, x, y, z):
    eq1 = (np.cos(theta_1)) * (np.cos(theta_2)  +  np.cos(theta_2 + theta_3)) - x
    eq2 = (np.sin(theta_1)) * (np.cos(theta_2)  +  np.cos(theta_2 + theta_3)) - y
    eq3 = np.sin(theta_2) + np.sin(theta_2 + theta_3) - z
    # print( f"eq1: {eq1} ; eq2: {eq2} ; eq3: {eq3}")
    if (np.abs(eq1) < threshold and np.abs(eq2) < threshold and np.abs(eq3) < threshold):
        return True
    return False

def compute_theta2(theta_1, theta_3, y, z):
    try:
        theta_2_solutions = [np.arctan2(z*np.sin(theta_1), y) -(theta_3/2), np.arctan2(z*np.sin(theta_1), y) - (theta_3/2) + np.pi, np.arctan2(z*np.sin(theta_1), y) - (theta_3/2) - np.pi]
    except Exception as e:
        print("error in computing for theta_2: ", e)
    return theta_2_solutions

def compute_theta1(x, y):
    theta_1_solutions = [np.arctan2(y, x), np.arctan2(y, x) - np.pi, np.arctan2(y, x) + np.pi]
    return theta_1_solutions

def compute_theta3(x, y, z):
    d = np.sqrt(x**2 + y**2 + z**2)
    acos_arg = (d**2 - 2) / 2
    if (acos_arg > 1 or acos_arg < -1):
        print("the end effector cannot reach to the given values of x, y, z... please try again with different values..")
        exit(1)
    theta_3_solutions = [np.arccos(acos_arg), -np.arccos(acos_arg)]
    return theta_3_solutions



def compute_theta_solutions():
    x, y, z = enter_end_effector_position()
    theta_1_solutions = compute_theta1(x, y)
    theta_3_solutions = compute_theta3(x, y, z)
    
    for theta_1 in theta_1_solutions:
        if (theta_1 < -np.pi or theta_1 > np.pi):
            continue

        for theta_3 in theta_3_solutions:
            if (theta_3 < -np.pi or theta_3 > np.pi):
                continue

            # compute theta_2 solutions
            solutions = compute_theta2(theta_1, theta_3, y, z)
            for theta_2 in solutions:
                # print(f"[{theta_1}, {theta_2}, {theta_3}]")
                if (theta_2 <= -np.pi or theta_2 >= np.pi):
                    continue

                # check whether computed theta values are indeed satisfying the equations or not
                solution_possible_or_not = check_solutions_possible(theta_1, theta_2, theta_3, x, y, z)
                if (not solution_possible_or_not):
                    continue

                # check for theta values within limits
                within_limit = False
                if (theta_2 > -np.pi/4 and theta_2 < 3* np.pi/4 and theta_3 > -3*np.pi/4 and theta_3 < 3*np.pi/4):
                    within_limit = True
                
                print(f"Angle Thetas solved: [{theta_1}, {theta_2}, {theta_3}] , solution possible: {within_limit}")
                print()

        
    
if __name__ == "__main__":
    compute_theta_solutions()




