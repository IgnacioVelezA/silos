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
import plotly.graph_objects as go

from scipy.signal import savgol_filter
import scipy.interpolate
# Polyfit from https://stackoverflow.com/questions/7997152/python-3d-polynomial-surface-fit-order-dependent

angulo_zero = 0#30*np.pi/180
#angulo_zero = 30*np.pi/180



#////plot_measure==================================================================================
def plot_measure(theta_list, phi_list, distance_measurements, minAxis, maxAxis, titlei = False):
    theta_list_rad = [value * np.pi/180 for value in theta_list]
    phi_list_rad = [value * np.pi/180 for value in phi_list]

    X = np.zeros(len(distance_measurements))
    Y = np.zeros(len(distance_measurements))
    Z = np.zeros(len(distance_measurements))
    index = list(np.zeros(len(distance_measurements)))

    distance_centered = distance_measurements#*np.cos(angulo_zero)

    for i in range(len(distance_measurements)):
        # X e Y están multiplicados por -1 mientras se hacen pruebas horizontales,
        # para implementación final debe ser positivo
        # theta = elevation; phi = azimutal
        X[i] = np.sin(theta_list_rad[i]) * np.sin(phi_list_rad[i]) * (distance_centered[i]+0.01)
        Y[i] = np.sin(theta_list_rad[i]) * np.cos(phi_list_rad[i]) * (distance_centered[i]+0.01)
        Z[i] = -distance_centered[i]*np.cos(theta_list_rad[i])
        index[i] = f'punto {i}; elev: {theta_list[i]}; azi: {phi_list[i]}; rawdist = {distance_measurements[i]}'
        if distance_measurements[i] > 30:
           Z[i] = -35

    # Configures the figure based on the chosen visualization
    fig = go.Figure()

    scatter = fig.add_trace(go.Scatter3d(
        x=X,
        y=Y,
        z=Z,
        mode='markers',
            marker=dict(
            size=3,
            color=Z,  # Usar la coordenada z como color
            colorscale='Viridis',  # Colormap
            colorbar=dict(title='Eje Z')
        ),
        text = index
        ))

    fig.update_layout(scene=dict(
        xaxis=dict(title='Eje X'),
        yaxis=dict(title='Eje Y'),
        zaxis=dict(title='Eje Z', range = [-maxAxis,minAxis]),
        ),title = titlei)
        
#fig.colorbar(scatter, shrink=0.5, aspect=5, label = "Distance [m]")

    fig.show()

    while True:
        mark = input('Indice (just enter to break):')
        if mark:
            #if mark == x:
            indxMark = int(mark)
            fig = go.Figure()
            scatter = fig.add_trace(go.Scatter3d(
                x=X,
                y=Y,
                z=Z,
                mode='markers',
                    marker=dict(
                    size=3,
                    color=Z,  # Usar la coordenada z como color
                    colorscale='Viridis',  # Colormap
                    colorbar=dict(title='Eje Z')
                ),
                text = index
                ))
            scatter = fig.add_trace(go.Scatter3d(
                x=[X[indxMark]],
                y=[Y[indxMark]],
                z=[Z[indxMark]],
                mode='markers',
                    marker=dict(
                    size=3,
                    color='red'
                ),
                text = mark
                ))

                # fig.update_layout(scene=dict(
                #     xaxis=dict(title='Eje X'),
                #     yaxis=dict(title='Eje Y'),
                #     zaxis=dict(title='Eje Z', range = [minAxis, maxAxis]),
                #     ),title = titlei)

            fig.show()
        else:
            break
    return [X, Y, Z]
#////END: plot_measure=============================================================================


#////plot_curve====================================================================================
def plot_curve(i, xs, thr, jump ,title = ''):
    curve = curves[i]
    distance = distanceFinder.distanceFinder(curve, thr ,MINDISTANCE,MAXDISTANCE, jump)
    z = np.cos(theta_angles[i]*np.pi/180)*distance


    plt.plot(xs, curve)
    plt.vlines(distance,0,50, label=f'radial={distance}')
    plt.xlabel('distance[m]')
    plt.ylabel('Power')
    plt.title(f'{title}, z = {z}')
    plt.legend()
    plt.show()
#////END: plot_curve===============================================================================


#////curveComparison===============================================================================
def curveComparison(crvindx):
    curve_og = curves[crvindx]
    curve_mean = np.zeros_like(curve_og)

    x = np.linspace(MINDISTANCE,MAXDISTANCE,128)

    curve_mean[0] = (curve_og[0]+curve_og[1])/3
    for i in range(len(curve_og)-2):
        curve_mean[i+1] = (curve_og[i]+curve_og[i+1]+curve_og[i+2])/3 

    i += 1
    curve_mean[i+1] = (curve_og[i]+curve_og[i+1])/3

    x_interpl = np.linspace(MINDISTANCE,MAXDISTANCE,2048)
    xstep = x_interpl[1] - x_interpl[0]

    xy_spline = scipy.interpolate.interp1d(x, curve_og)
    mean_spline = scipy.interpolate.interp1d(x, curve_mean)

    interpl_curve = xy_spline(x_interpl)
    interpl_meancurve = mean_spline(x_interpl)

    cubicSPCurve = scipy.interpolate.CubicSpline(x, curve_og)
    pChipCurve = scipy.interpolate.PchipInterpolator(x, curve_og)    

    max_og = np.argmax(curve_og)*int(2048/128)*xstep
    max_cubic = np.argmax(cubicSPCurve(x_interpl))*xstep
    max_pChip = np.argmax(pChipCurve(x_interpl))*xstep

    print(max_og)
    print(max_cubic)
    print(max_pChip)

    fig1 = plt.figure()
    plt.plot(x, curve_og, 'k*')
    plt.plot(x_interpl, interpl_curve, 'r')
    plt.plot(x_interpl, interpl_meancurve, 'y')
    plt.plot(x_interpl, cubicSPCurve(x_interpl), 'g')   
    #plt.plot(x_interpl, pChipCurve(x_interpl), 'm')
    
    #plt.vlines(max_og,0,50,'k', label=f'original max')
    #plt.vlines(max_cubic,0,50,'g', label=f'max w Cubic')
    #plt.vlines(max_pChip,0,50,'b', label=f'max w pChip')

    plt.xlabel('distance[m]')
    plt.ylabel('Power')
    plt.legend()
    plt.show()


#////END: curveComparison==========================================================================


#////plotCut ======================================================================================
def plotCut(XYZcoords, spline, xcut = False, ycut = False, step_angle = 3):
    Xcoords, Ycoords, Zcoords = XYZcoords[spline]
    n_points = len(Zcoords)  
    Xcuts= []
    Ycuts= []
    Zcuts= []
    indexCuts = []
    if (xcut is False) and (ycut is False):
        print ('Select at least one axis')
        return False

    elif not (xcut is False): #Fixed value of X then a cut parallel to y axis
        for pi in range(n_points):
            Xpi = Xcoords[pi]
            if (Xpi <= xcut[1]) and (Xpi >= xcut[0]):
                Xcuts.append(Xpi)
                Ycuts.append(Ycoords[pi])
                Zcuts.append(Zcoords[pi])
                indexCuts.append(pi)

    elif not (ycut is False): #Fixed value of X then a cut parallel to y axis
        for pi in range(n_points):
            Ypi = Ycoords[pi]
            if (Ypi <= ycut[1]) and (Ypi >= ycut[0]):
                Xcuts.append(Xcoords[pi])
                Ycuts.append(Ypi)
                Zcuts.append(Zcoords[pi])    
                indexCuts.append(pi)

    fig2 = go.Figure()
    scatter = fig2.add_trace(go.Scatter3d(
        x=Xcuts,
        y=Ycuts,
        z=Zcuts,
        mode='markers',
            marker=dict(
            size=3,
            color=Z,  # Usar la coordenada z como color
            colorscale='Viridis',  # Colormap
            colorbar=dict(title='Eje Z')
        ),
        text = indexCuts
        ))

    fig2.update_layout(scene=dict(
        xaxis=dict(title='Eje X'),
        yaxis=dict(title='Eje Y'),
        zaxis=dict(title='Eje Z', range = [0, 30])
        ),title = spline)
        
    fig2.show()
    return Xcuts, Ycuts, Zcuts
        

#////END: plotCut =================================================================================


#////interpolateBetween ===========================================================================
def interpolateBetween(XYZcoords, minAxis, maxAxisvisualization = "3d", 
                                  iteration = 0, poly_order = 3, n_mesh=50):

    X = XYZcoords[0]
    Y = XYZcoords[1]
    Z = XYZcoords[2]
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

    ax = fig.add_subplot(1,1,1, projection='3d')
    ax.set(zlim=(minAxis, maxAxis))
    vmin = min(np.min(valid_interZ_array), np.min(Z))
    vmax = max(np.max(valid_interZ_array), np.max(Z))
    plot = ax.scatter(valid_interX_array, valid_interY_array, valid_interZ_array, c = valid_interZ_array,
                        cmap = "gist_rainbow", vmin=vmin, vmax=vmax, alpha=0.5)
    plot = ax.scatter(X,Y,Z, c = Z, cmap="gist_rainbow",marker="x", vmin=vmin, vmax=vmax)
    

    fig.colorbar(plot, shrink=0.5, aspect=5, label = "Distance [m]")

    plt.show()
    return [X, Y, Z]

#////END: interpolateBetween ===========================================================================


if __name__=='__main__':
    parser = argparse.ArgumentParser(description='plots a silos measurement'+
                                                 ' from a pkl file')
    parser.add_argument('filename', help='name of the file')
    parser.add_argument('-min', '--MINDISTANCE', default = 0, type = int)
    parser.add_argument('-max', '--MAXDISTANCE', default = 30, type = int)
    parser.add_argument('-thr', '--threshold', default = 15, type = int)

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
    threshold = args.threshold
    MINDISTANCE = args.MINDISTANCE
    MAXDISTANCE = args.MAXDISTANCE

    xs = np.array(np.arange(start=MINDISTANCE, stop=MAXDISTANCE, step=(MAXDISTANCE-MINDISTANCE)/128))

    try:
        file = open(fileDir,'rb')
    except:
        print("File does not exist")

    # reading file and saving distances
    traj_angle_dict = 0
    distances_date_tuple_list = []
    file_len_counter = 0
    while True:
        try:
            if file_len_counter == 0:
                traj_angle_dict = pkl.load(file)
                print(traj_angle_dict)
            else:
                real_traj = pkl.load(file)
                distances_date_tuple_list.append(pkl.load(file))
            file_len_counter += 1
        except:
            break
    file.close()

    # saving target angles
    n_iterations = file_len_counter-1
    print(distances_date_tuple_list)
    n_points = len(distances_date_tuple_list[0]) #<------------cambiar para determinar n de puntos

    phi_angles = np.zeros(n_points)
    theta_angles = np.zeros(n_points)

    for i in range(n_points):
        phi_angles[i] = traj_angle_dict["traj"][i][0]
        theta_angles[i] = traj_angle_dict["traj"][i][1]

    distances = np.zeros((n_iterations,n_points)) # [iter][distance]
    curves = []

    titles = ['mean']
    XYZsplines = {}

    for i in range(len(titles)):
        angle_rep = 0 #<-- debe poder leerse desde la data
        for j in range(n_points):
            curves_jpoint = np.array(distances_date_tuple_list[0][j][1]) #
            distances[0][j] = distanceFinder.distanceSplines(curves_jpoint, threshold,MINDISTANCE,MAXDISTANCE, 3, 1)
            curves.append(curves_jpoint)

            # distances[i] corresponds to the measured distances for the i-th iteration
            # plots using the visualization set as argument

        [X, Y, Z] = plot_measure(theta_angles, phi_angles, distances[0], MINDISTANCE,MAXDISTANCE, titlei = titles[i])
        
        XYZsplines[titles[i]] = [X,Y,Z]

    # curves[i][j] corresponds to the curve of the j-th point of the i-th iteration
