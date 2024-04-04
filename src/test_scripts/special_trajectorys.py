import numpy as np

def linear_traj(az_angle, el_step, el_max = 30, el_min = -30):
    n_points = int(np.floor((el_max - el_min) / el_step))
    traj = np.zeros((n_points+1,2))
    traj[0] = (az_angle, el_min)
    for i in range(n_points):
        current_step = el_min + el_step*(i + 1)
        traj[i+1] = (az_angle, current_step)

    return traj