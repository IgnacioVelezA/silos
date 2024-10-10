import numpy as np
import argparse
def sexto_de_aro(dist_cent, nivel):
    radio_i = dist_cent/2
    radio_0 = 2*radio_i/np.sqrt(3)
    center_n = [(0, 2*nivel*radio_i)]
    
    if nivel >= 2:
        for c in range(nivel-1):
            center_x = center_n[c][0] + radio_0 + radio_i/2
            center_y = center_n[c][1] - radio_i
            center_n.append((center_x, center_y))
    
    return center_n

def aro_completo(dist_cent, nivel):
    centers_completo = sexto_de_aro(dist_cent, nivel)
    largo_init = len(centers_completo)

    for i in range(5):
        for c in range(largo_init):
            ci = c + i*(nivel)
            x_rotado = centers_completo[ci][0]/2 - centers_completo[ci][1]*(-np.sqrt(3)/2)
            y_rotado = centers_completo[ci][0]*(-np.sqrt(3)/2) + centers_completo[ci][1]/2
            x_rotado = round(x_rotado, 4)
            y_rotado = round(y_rotado, 4)
            centers_completo.append((x_rotado,y_rotado))

    return centers_completo

def all_levels(dist_cent, total_levels):
    centers_for_level = []

    for n in range(total_levels-1):
        level_n = aro_completo(dist_cent, n+1)

        for center in level_n:
            centers_for_level.append(center)

    return centers_for_level

def hex_trajectory(dist_cent, rmax, set_offset_el = True, offset_elev_degree = 10):
    """
    Returns a tuple list with az and el points of a hexagonal trajectory
    to measure given radio0 wich defines the radar's HPBW, and a rmax
    wich is the radius of the surface intended to measure. 
    """
    radio_i = dist_cent/2
    max_level = int(np.floor((rmax - radio_i)/(2*radio_i))) + 1
    puntos= all_levels(dist_cent, max_level)
    trajectory = [(0,0)]

    for vector in puntos:
        x = vector[0]
        y = vector[1]

        if x == 0 and y > 0:
            az = 90
        elif x == 0 and y < 0:
            az = -90
        else:
            az = round(np.arctan(y/x)*180/np.pi, 2)

        if x < 0:
            el = round(-np.sqrt(x**2 + y**2), 4)
        else:
            el = round(np.sqrt(x**2 + y**2), 4)
        trajectory.append((az, el))

    if set_offset_el == True:
        trajectory = offset_elev(trajectory, offset_elev_degree)

    return trajectory   #, puntos

def offset_elev(trajectory, offset_degree = 5.0):
    for vector in range(len(trajectory)):
        vector_az = trajectory[vector][0]
        vector_el = trajectory[vector][1]
        if vector_el > 0.0:
            trajectory[vector] = (vector_az, vector_el-offset_degree)
    return trajectory
