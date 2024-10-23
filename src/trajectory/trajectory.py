"""
Creation of different trayectories for the silos measurements, currently only standard13 and square_mesh are supported
All of the trayectories should return a list of tuples for each point of the trayectory,
each tuple should have 2 elements, being the first one the azimuth angle and the second one the elevation angle
"""

import math
import numpy as np
import matplotlib.pyplot as plt

def standard13(max_angle):
    """
    Returns a tuple list with az and el points to measure, max_angle corresponds
    to the angle between the zenith and the lowest point to measure
    a prototype trajectory to measure 13 points
    """
    # (90 - elevation) actually
    target_el_angles_list = []
    target_az_angles_list = []

    #Point 1
    target_az_angles_list.append(int(0))
    target_el_angles_list.append(int(0))

    #Point 2
    target_az_angles_list.append(int(45))
    target_el_angles_list.append(int(max_angle/2))

    #Point 3
    target_az_angles_list.append(int(-45))
    target_el_angles_list.append(int(max_angle/2))

    #Point 4
    target_az_angles_list.append(int(-45))
    target_el_angles_list.append(int(-max_angle/2))

    #Point 5
    target_az_angles_list.append(int(45))
    target_el_angles_list.append(int(-max_angle/2))

    #Point 6
    target_az_angles_list.append(int(0))
    target_el_angles_list.append(int(-max_angle/math.sqrt(2)))

    #Point 7
    target_az_angles_list.append(int(-90))
    target_el_angles_list.append(int(max_angle/math.sqrt(2)))

    #Point 8
    target_az_angles_list.append(int(0))
    target_el_angles_list.append(int(max_angle/math.sqrt(2)))

    #Point 9
    target_az_angles_list.append(int(90))
    target_el_angles_list.append(int(max_angle/math.sqrt(2)))

    #Point 10
    target_az_angles_list.append(int(45))
    target_el_angles_list.append(int(max_angle))

    #Point 11
    target_az_angles_list.append(int(-45))
    target_el_angles_list.append(int(max_angle))

    #Point 12
    target_az_angles_list.append(int(-45))
    target_el_angles_list.append(int(-max_angle))

    #Point 13
    target_az_angles_list.append(int(45))
    target_el_angles_list.append(int(-max_angle))

    return_list = []

    for i in range(13):
        return_list.append((target_az_angles_list[i], target_el_angles_list[i]))
    
    return return_list

def n_times_n_mesh(max_angle, n, angle_repetition = 1):
    """
    Returns a tuple list with the az and el points to measure, max_angle corresponds
    to the angle between the zenith and the lowest point to measure
    n corresponds to the number of points in one dimensions of the mesh
    angle_repetition corresponds to the number of time to measure in one direction
    """
    # (90 - elevation) actually
    target_el_angles_list = []
    target_az_angles_list = []

    neg_el_target_el_angles_list = []
    neg_el_target_az_angles_list = []
    if n == 1:
        return [(0,0)]

    # For even n
    elif n % 2 == 0:
        #d is the cartesian separation between points in the same row or column
        d = (max_angle*2/(n-1)) / math.sqrt(2)

        q = n/2 + 0.5
   
    # For uneven n
    elif n % 2 == 1:
        #d is the cartesian separation between points in the same row or column
        d = (max_angle/ (n//2)) / math.sqrt(2)

        q = n // 2 + 1

    for i in range(1,n+1):
        for j in range (1,n+1):
            theta = math.sqrt((i-q)**2 + (j-q)**2)*d
            if i == j and i == q:
                phi = 90
                target_az_angles_list.append(phi)
                target_el_angles_list.append(theta)
            elif i >= q and j <= q and i + j >= 2 * q:
                phi = 45 + math.atan2(abs(j-q),abs(i-q))*180/math.pi

                if i + j != 2 * q:
                    target_az_angles_list.append(phi)
                    target_el_angles_list.append(theta)
                    neg_el_target_az_angles_list.append(phi)
                    neg_el_target_el_angles_list.append(-1*theta)
                else:
                    target_az_angles_list.append(phi)
                    target_el_angles_list.append(theta)

            elif i >= q and j >= q and i >= j:
                phi = 45 - math.atan2(abs(j-q),abs(i-q))*180/math.pi

                target_az_angles_list.append(phi)
                target_el_angles_list.append(theta)
                neg_el_target_az_angles_list.append(phi)
                neg_el_target_el_angles_list.append(-1*theta)

            elif i >= q and j >= q and j >= i:
                phi = -45 + math.atan2(abs(i-q),abs(j-q))*180/math.pi

                target_az_angles_list.append(phi)
                target_el_angles_list.append(theta)
                neg_el_target_az_angles_list.append(phi)
                neg_el_target_el_angles_list.append(-1*theta)

            elif i <= q and j >= q and i + j >= 2 * q:
                phi = -45 - math.atan2(abs(i-q),abs(j-q))*180/math.pi
                
                if i + j != 2 * q:
                    target_az_angles_list.append(phi)
                    target_el_angles_list.append(theta)
                    neg_el_target_az_angles_list.append(phi)
                    neg_el_target_el_angles_list.append(-1*theta)
                else:
                    target_az_angles_list.append(phi)
                    target_el_angles_list.append(theta)

    target_az_angles_list = target_az_angles_list + neg_el_target_az_angles_list
    target_el_angles_list = target_el_angles_list + neg_el_target_el_angles_list

    tuple_list = []

    for i in range(n*n):
        tuple_list.append((int(target_az_angles_list[i]*100)/100, int(target_el_angles_list[i]*100)/100))

    # Calculating the most efficient path
    #
    # We decided to go for the "spiral" path...
    aux_tuple_list = tuple_list

    angle_error = 1

    theta_defining_tuples = []
    for i in tuple_list:

        # If the point is the theta = 90 degrees line and in positive theta
        if abs(i[0] - 90) <= angle_error and i[1] >= 0:
            theta_defining_tuples.append(i)

    theta_defining_tuples.sort(key=lambda a: a[1])

    #print(theta_defining_tuples)
    optimum_list = []
    
    if abs(theta_defining_tuples[0][1] - 0) <= angle_error:
        aux_tuple_list.remove(theta_defining_tuples[0])
        for j in range(angle_repetition):
            optimum_list.append(theta_defining_tuples[0])
        current_theta_index = 1
    else:
        current_theta_index = 0


    while len(aux_tuple_list) != 0:
        current_theta = theta_defining_tuples[current_theta_index][1]
        #print(current_theta)

        possible_targets_pos_el = []
        possible_targets_neg_el = []

        for i in aux_tuple_list:
            #print(i)
            if abs(i[1]) <= current_theta + angle_error:
                
                if i[1] > 0:
                    possible_targets_pos_el.append(i)
                else:
                    possible_targets_neg_el.append(i)

        possible_targets_pos_el.sort(key=lambda a: -a[0])
        possible_targets_neg_el.sort(key=lambda a: -a[0])

        #print(aux_tuple_list)

        for i in range(len(possible_targets_pos_el)):
            for j in range(angle_repetition):
                optimum_list.append(possible_targets_pos_el[i])
            aux_tuple_list.remove(possible_targets_pos_el[i])

        for i in range(len(possible_targets_neg_el)):
            for j in range(angle_repetition):
                optimum_list.append(possible_targets_neg_el[i])
            aux_tuple_list.remove(possible_targets_neg_el[i])

        current_theta_index = current_theta_index + 1

    # Returning spiral path
    return_list = optimum_list
    return return_list

def circle_path(initial_circle_points, theta_max, n_levels):
    """
    Returns a tuple list with the az and el points to measure, max_angle corresponds
    to the angle between the zenith and the lowest point to measure
    initial_circle_points corresponds to the points to measure in the first circle
    theta_max corresponds to the maximum elevation
    n_levels corresponds to the number of circles to measure
    """
    target_el_angles_list = []
    target_az_angles_list = []

    if n_levels <= 1:
        target_el_angles_list.append(0)
        target_az_angles_list.append(0)
    else:
        theta_step = theta_max/(n_levels-1)
        target_el_angles_list.append(0)
        target_az_angles_list.append(0)
        for i in range(n_levels):
            theta = i*theta_step
            for j in range(i*initial_circle_points):
                phi = -90+j*360/(i*initial_circle_points)
                phi_target = phi
                theta_target = theta

                if phi > 90:
                    phi_target = phi-180
                    theta_target = -theta
                
                target_el_angles_list.append(theta_target)
                target_az_angles_list.append(phi_target)

    return_list = []
    for i in range(len(target_el_angles_list)):
        return_list.append((int(target_az_angles_list[i]*100)/100, int(target_el_angles_list[i]*100)/100))

    return return_list


def plot_traj(trayectoria):
    theta_list_rad = [value[1] * np.pi/180 for value in trayectoria]
    phi_list_rad = [value[0] * np.pi/180 for value in trayectoria]

    X = np.zeros(len(trayectoria))
    Y = np.zeros(len(trayectoria))

    for i in range(len(trayectoria)):
        # X e Y están multiplicados por -1 mientras se hacen pruebas horizontales,
        # para implementación final debe ser positivo
        # theta = elevation; phi = azimutal
        X[i] = np.sin(theta_list_rad[i]) * np.sin(phi_list_rad[i])
        Y[i] = np.sin(theta_list_rad[i]) * np.cos(phi_list_rad[i])

    plt.figure()
    plt.scatter(X,Y)
    plt.show()
