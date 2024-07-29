import pickle as pkl
import numpy as np
import argparse
import matplotlib.pyplot as plt
import itertools
import csv, ipdb
from shapely.geometry import Point
from geopandas import GeoSeries
from src.test_scripts import distanceFinder
import os

# Polyfit from https://stackoverflow.com/questions/7997152/python-3d-polynomial-surface-fit-order-dependent

angulo_zero = 0#30*np.pi/180
#angulo_zero = 30*np.pi/180



#////plot_measure==================================================================================
def plot_measure(theta_list, phi_list, distance_measurements, save, name, date, minAxis, maxAxis,, visualization = "3d"):
    theta_list_rad = [value * np.pi/180 for value in theta_list]
    phi_list_rad = [value * np.pi/180 for value in phi_list]

    X = np.zeros(len(distance_measurements))
    Y = np.zeros(len(distance_measurements))
    Z = np.zeros(len(distance_measurements))

    distance_centered = distance_measurements*np.cos(angulo_zero)

    for i in range(len(distance_measurements)):
        # X e Y están multiplicados por -1 mientras se hacen pruebas horizontales, 
        # para implementación final debe ser positivo
        # theta = elevation; phi = azimutal
        X[i] = np.sin(theta_list_rad[i]) * np.sin(phi_list_rad[i]) * (distance_centered[i]+0.01)
        Y[i] = -1*np.sin(theta_list_rad[i]) * np.cos(phi_list_rad[i]) * (distance_centered[i]+0.01)
        Z[i] = np.cos(theta_list_rad[i])*distance_centered[i]
        
        if Z[i] == 35:
           Z[i] = 35

        #print(f'(Az[{i}] = {round(phi_list[i])}, El[{i}] = {round(theta_list[i])}); (X[{i}] = {round(X[i], 3)}, Y[{i}] = {round(Y[i], 3)})')
    
    # Configures the figure based on the chosen visualization
    fig = plt.figure()

    # max_Y_range = max(Y)
    # graphLimit = max([max_X_range,max_Y_range])

    if visualization == "3d":
        ax = fig.add_subplot(1,1,1, projection='3d')
        ax.set_box_aspect([1,1,1])
        ax.set_ylabel('y axis(Vertical)')
        ax.set_xlabel('x axis (Horizontal)')
        scatter = ax.scatter(X,Y,Z, c=Z, cmap="gist_rainbow")
        fig.colorbar(scatter, shrink=0.5, aspect=5, label = "Distance [m]")


    else:
        plt.scatter(X,Y,c=Z, cmap="gist_rainbow",alpha=1)
        plt.gca().set_aspect('equal', adjustable= 'box')
        plt.colorbar(label="Distance [m]", orientation="vertical")
        plt.savefig('blabla.png', transparent = True)

        if save == True:
            savingDir = 'results/' + date 
            try:
                os.mkdir(savingDir)
            except:
                print(f'{date} file already exist')
            
            savingName = savingDir + '/plot_' + vis + '_' + name + date +'.png'
            plt.savefig(savingName,transparent = True)

    #plot = ax.plot_trisurf(X, Y, Z)
    plt.show()
    return [X, Y, Z]
#////END: plot_measure=============================================================================


#////plot_measure_and_interpolate==================================================================
def plot_measure_and_interpolate(theta_list, phi_list, distance_measurements, 
                                  minAxis, maxAxisvisualization = "3d", 
                                  iteration = 0, poly_order = 3, n_mesh=50):
    theta_list_rad = [value * np.pi/180 for value in theta_list]
    phi_list_rad = [value * np.pi/180 for value in phi_list]

    distance_list = distance_measurements[iteration]
    X = np.zeros(len(distance_list))
    Y = np.zeros(len(distance_list))
    Z = np.zeros(len(distance_list))

    for i in range(len(distance_list)):
        Xi = np.sin(theta_list_rad[i]) * np.cos(phi_list_rad[i]) * distance_list[i]
        Yi = np.sin(theta_list_rad[i]) * np.sin(phi_list_rad[i]) * distance_list[i]
        Zi = distance_list[i] * np.cos(theta_list_rad[i])

        # Rotación considerando elevación de radar de angulo_zero 
        X[i] = Xi*np.cos(17*np.pi/180) - Zi*np.sin(17*np.pi/180)
        Y[i] = Yi
        if distance_list[i] == 35:
            Z[i] = 35#Zi
        else:
            Z[i] = Xi*np.sin(17*np.pi/180) + Zi*np.cos(17*np.pi/180)
    # Fit a 3rd order, 2d polynomial
    m = polyfit2d(X,Y,Z, order = poly_order)

    # Evaluate it on a grid...
    nx, ny = n_mesh, n_mesh
    xx, yy = np.meshgrid(np.linspace(X.min(), X.max(), nx), 
                         np.linspace(Y.min(), Y.max(), ny))
    positions = np.vstack([xx.ravel(), yy.ravel()])

    mesh_points_list = []

    for i in range(len(positions[0])):
        mesh_points_list.append(Point(positions[0][i],positions[1][i]))

    # Calculates the valid zone for points
    geos = GeoSeries(map(Point, zip(X, Y)))
    valid_2d_zone = geos.unary_union.convex_hull
    valid_points_list = []

    # Calculating valid meshgrid points
    for i in range(len(mesh_points_list)):
        if valid_2d_zone.intersects(mesh_points_list[i]):
            valid_points_list.append(mesh_points_list[i])

    # Constructing new interpolated array
    valid_interX_array = []
    valid_interY_array = []

    for i in range(len(valid_points_list)):
        valid_interX_array.append(valid_points_list[i].x)
        valid_interY_array.append(valid_points_list[i].y)

    # Calculating Z value for valid grid points
    zz = polyval2d(xx, yy, m)
    
    valid_interZ_array = []
    for i in range(len(valid_interX_array)):
        x_index = np.where(xx==valid_interX_array[i])[1][0]
        y_index = np.where(yy==valid_interY_array[i])[0][1]
        valid_interZ_array.append(zz[y_index][x_index])

    # Configures the figure based on the chosen visualization
    fig = plt.figure()
    if visualization == "3d":
        ax = fig.add_subplot(1,1,1, projection='3d')
        ax.set(zlim=(minAxis, maxAxis))
        vmin = min(np.min(valid_interZ_array), np.min(Z))
        vmax = max(np.max(valid_interZ_array), np.max(Z))
        plot = ax.scatter(valid_interX_array, valid_interY_array, valid_interZ_array, c = valid_interZ_array,
                          cmap = "gist_rainbow", vmin=vmin, vmax=vmax, alpha=0.5)
        plot = ax.scatter(X,Y,Z, c = Z, cmap="gist_rainbow",marker="x", vmin=vmin, vmax=vmax)
        

        fig.colorbar(plot, shrink=0.5, aspect=5, label = "Distance [m]")

    else:
        
        # Determine the common value range for the colorbars
        vmin = min(np.min(valid_interZ_array), np.min(Z))
        vmax = max(np.max(valid_interZ_array), np.max(Z))
        plt.scatter(valid_interX_array, valid_interY_array, c = valid_interZ_array, 
                    cmap="gist_rainbow", vmin=vmin, vmax=vmax, alpha=0.5)
        plt.scatter(X, Y, c = Z, cmap="gist_rainbow", marker="x", vmin=vmin, vmax=vmax,alpha=1)
        
        # HAY QUE REVISAR QUE LA ESCALA DE AMBOS PLOT SEA LA MISMA EN COLORES!!!
        plt.colorbar(label="Distance [m]", orientation="vertical")

        plt.gca().set_aspect('equal')

    plt.show()
    return [X, Y, Z]
#////END: plot_measure_and_interpolate=============================================================



#////save_plot_png=================================================================================
def save_plot_png(self, name, date, vis):
    savingDir = 'results/' + date 
    try:
        os.mkdir(savingDir)
    except:
        print(f'{date} file already exist')
    
    savingName = savingDir + '/plot_' + vis + '_' + name + date +'.png'
    self.savefig(savingName,transparent = True)
    return
#////END: save_plot_png============================================================================


#////polyfit2d=====================================================================================
def polyfit2d(x, y, z, order=3):
    ncols = (order + 1)**2
    G = np.zeros((x.size, ncols))
    ij = itertools.product(range(order+1), range(order+1))
    for k, (i,j) in enumerate(ij):
        G[:,k] = x**i * y**j
    m, _, _, _ = np.linalg.lstsq(G, z, rcond = None)
    return m
#////END: polyfit2d================================================================================


#////polyval2d=====================================================================================
def polyval2d(x, y, m):
    order = int(np.sqrt(len(m))) - 1
    ij = itertools.product(range(order+1), range(order+1))
    z = np.zeros_like(x)
    for a, (i,j) in zip(m, ij):
        z += a * x**i * y**j
    return z
#////END: polyval2d================================================================================


#////plotBetween===================================================================================
def plotBetween(theta, phi, distances_input, min, max, interpolate = False,
                   visualization = "3d",iteration = 0,
                                 poly_order = 3, n_mesh=50):
    new_distances = []
    new_theta = []
    new_phi = []

    for punto_i in range(len(distances_input)):
        if distances_input[punto_i] <= max and distances_input[punto_i] >= min:
            new_distances.append(distances_input[punto_i])
            new_theta.append(theta[punto_i])
            new_phi.append(phi[punto_i])
    new_distances = np.array(new_distances)
    if not interpolate:
        plot_measure(new_theta, new_phi, new_distances,save, name, date, visualization = "3d", minAxis = min, maxAxis = max)

    else:
        plot_measure_and_interpolate(new_theta, new_phi, new_distances,save, name, date, visualization = "3d", 
                                     iteration=0, poly_order=3, n_mesh=50, minAxis = min, maxAxis = max)
#////END: plotBetween==============================================================================
       


if __name__=='__main__':
    parser = argparse.ArgumentParser(description='plots a silos measurement'+
                                                 ' from a pkl file')
    parser.add_argument('filename', help='name of the file')
    parser.add_argument('-v', '--visualization', help="how to visualize the data, '3d' or 'color'. 3d by default")
    parser.add_argument('--interpolation', action='store_true',help="write this if you want to interpolate", default=False) 
    parser.add_argument('--no-interpolation', dest='interpolation', action='store_false', help="write this if you DON'T want to interpolate")
    parser.add_argument('-min', '--MINDISTANCE', default = 0, type = int)
    parser.add_argument('-max', '--MAXDISTANCE', default = 30, type = int)
    parser.add_argument('-thr', '--threshold', default = 15, type = int)   
    parser.add_argument('-m2f', '--target_measure_2fit', help="choose the target measure iteration to fit. 1 by default",
                         default = 0, type = int)
    parser.add_argument('-n_mesh', '--n_mesh_size', help="choose n size of the nxn meshgrid for data interpolation",
                         default = 50, type = int)
    parser.add_argument('-poly_order', '--polyfit_order', help="order of the polynomial fit",
                          default = 3, type = int)
    parser.add_argument('--save', action='store_true',help="write this if you want to save the plot", default=False) 
    parser.add_argument('--no-save', dest='save', action='store_false', help="write this if you DON'T want to save the plot")

    # parser.add_argument('--smoothing', action='store_true',help="write this if you want to smooth the curve", default= False)
    # parser.add_argument('--no-smoothing', dest='smoothing', action='store_false', help="write this if you DON'T want to smooth the curve")
    args = parser.parse_args()

    if args.filename[-4:] == '.pkl':
        name = args.filename[:-14]
        date = args.filename[-14:-4]
        fileDir = 'measurements/' + date + '/'
        fileDir = fileDir + args.filename
    else:
        name = args.filename[:-10]
        date = args.filename[-10:]
        fileDir = 'measurements/' + date +'/'
        fileDir = fileDir + args.filename + '.pkl'

    # saving arguments to variables
    vis = args.visualization
    interpolate = args.interpolation
    MINDISTANCE = args.MINDISTANCE
    MAXDISTANCE = args.MAXDISTANCE
    threshold = args.threshold
    target_measure_2fit = args.target_measure_2fit
    polyfit_order = args.polyfit_order
    n_mesh_size = args.n_mesh_size
    save = args.save 
    
    xs = np.array(np.arange(start=MINDISTANCE, stop=MAXDISTANCE, step=MAXDISTANCE/128))
    
    try:
        file = open(fileDir,'rb')
    except:
        print("File does not exist")

    # reading file and saving distances
    traj_angle_dict = 0
    real_trajectory = []
    distances_date_tuple_list = []
    file_len_counter = 0
    while True:
        try:
            if file_len_counter == 0:
                traj_angle_dict = pkl.load(file)
                #print(traj_angle_dict)
            else:
                real_trajectory.append(pkl.load(file))
                distances_date_tuple_list.append(pkl.load(file))
                print('--------------')
                print(real_trajectory)
                print('--------------')
                print(distances_date_tuple_list)

            file_len_counter += 1
        except:
            break
    file.close()

    # saving target angles
    n_iterations = file_len_counter-1
    print(n_iterations)
    n_points = len(distances_date_tuple_list[0]) #<------------cambiar para determinar n de puntos

    phi_angles = np.zeros(n_points)
    theta_angles = np.zeros(n_points)

    for i in range(n_points):
        print(i)
        phi_angles[i] = traj_angle_dict["traj"][i][0]
        theta_angles[i] = traj_angle_dict["traj"][i][1]

    distances = np.zeros((n_iterations,n_points)) # [iter][distance]
    curves = []
    
    angle_rep = 0 #<-- debe poder leerse desde la data
    for j in range(n_points):
        curves_jpoint = np.array(distances_date_tuple_list[0][j][0][1]) #
        distances[0][j] = distanceFinder.distanceFinder(curves_jpoint, threshold)
        curves.append(curves_jpoint) 

    # checking chosen mesh size
    if n_mesh_size <= 0:
        print("Invalid mesh size, choosing n_mesh_size = 50 instead.")
        n_mesh_size = 50

    # checking chosen polynomial order
    if polyfit_order <= 0:
        print("Invalid polynomial fit order, choosing polyfit_order = 3 instead")
        polyfit_order = 3

    # checking target iteration to fit
    if target_measure_2fit < 1 or target_measure_2fit > n_iterations:
        print("Target measure to fit is not valid. Choosing target measure to 1 instead")
        target_measure_2fit = 1
    iteration_2fit = target_measure_2fit - 1

    # distances[i] corresponds to the measured distances for the i-th iteration
    # plots using the visualization set as argument
    if interpolate == True:
        [X, Y, Z] = plot_measure_and_interpolate(theta_angles, phi_angles, distances, 
                                    MINDISTANCE, MAXDISTANCE, visualization = vis, iteration=iteration_2fit, poly_order=polyfit_order, n_mesh=n_mesh_size)
    else:
        [X, Y, Z] = plot_measure(theta_angles, phi_angles, distances[0], save, name, date, MINDISTANCE,MAXDISTANCE, visualization = vis)

    # curves[i][j] corresponds to the curve of the j-th point of the i-th iteration
    #plot_curve(distances, 1,10)
    #print(distances)

