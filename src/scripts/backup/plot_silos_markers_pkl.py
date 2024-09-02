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
from scipy.spatial import Delaunay

from scipy.signal import savgol_filter
from scipy.spatial import Delaunay
import scipy.interpolate
# Polyfit from https://stackoverflow.com/questions/7997152/python-3d-polynomial-surface-fit-order-dependent

angulo_zero = 0#30*np.pi/180
#angulo_zero = 30*np.pi/180



#////plot_measure==================================================================================
def plot_measure(theta_list, phi_list, distance_measurements, minAxis, maxAxis, titlei = False):
    theta_list_rad = [value * np.pi/180 for value in theta_list]
    phi_list_rad = [value * np.pi/180 for value in phi_list]

    num_of_points = len(distance_measurements)

    X = np.zeros(num_of_points)
    Y = np.zeros(num_of_points)
    Z = np.zeros(num_of_points)
    index = list(np.zeros(num_of_points))

    distance_centered = distance_measurements#*np.cos(angulo_zero)

    vertices = np.zeros([num_of_points,3])

    for i in range(num_of_points):
        # X e Y están multiplicados por -1 mientras se hacen pruebas horizontales,
        # para implementación final debe ser positivo
        # theta = elevation; phi = azimutal
        X[i] = np.sin(theta_list_rad[i]) * np.sin(phi_list_rad[i]) * (distance_centered[i]+0.01)
        Y[i] = np.sin(theta_list_rad[i]) * np.cos(phi_list_rad[i]) * (distance_centered[i]+0.01)
        Z[i] = -distance_centered[i]*np.cos(theta_list_rad[i])
        index[i] = f'punto {i}; elev: {theta_list[i]}; azi: {phi_list[i]}; rawdist = {distance_measurements[i]}'

        if distance_measurements[i] > 30:
           Z[i] = -35

        vertices[i] = np.array([X[i], Y[i], Z[i]])

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
    return vertices
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

def correct_real_traj(traj_measured):
    offset_cero_azimutal = traj_measured[0][0]
    offset_cero_elev = traj_measured[0][1]
    real_traj_corr = []

    for punto_i in range(len(real_traj)):
        azimutal_encoder = (offset_cero_azimutal - traj_measured[punto_i][0] ) * 90/1024
        elevation_encoder = (offset_cero_elev - traj_measured[punto_i][1]) * 90/1024
        real_traj_corr.append((azimutal_encoder, elevation_encoder))
    return real_traj_corr

#////plot_with_encoder ===============================================================================
def plot_with_encoder(traj_measured, distance_measurements, minAxis, maxAxis, titlei = False):
    real_traj_corr = correct_real_traj(traj_measured)
    print(real_traj_corr)
    theta_list_rad = [value[1] * np.pi/180 for value in real_traj_corr]
    phi_list_rad = [value[0] * np.pi/180 for value in real_traj_corr]

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
        index[i] = f'punto {i}; elev: {real_traj_corr[i][1]}; azi: {real_traj_corr[i][0]}; rawdist = {distance_measurements[i]}'
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
        ),title = "Using encoders")
        
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

# def volume_calculator(xyzpoints):
#     X,Y,Z = XYZcoords
#     xypoints = np.array([X,Y])
#     tri.delaunay(xypoints)
#     indices = tri.simplices
#     volume = 0
#     for triangle_i in indices:
        
# def prism_volume_calculator(vertices): #vertices contains x,y,z of the top vertices
#     ver1,ver2,ver3 = vertices
#     mean_altitude = (ver1[2]+ver2[2]+ver3[2])/3
#     triangle_area = 


def tesselation(X,Y):
    xypoints = []

    for coord_i in range(len(X)):
        xypoints.append([X[coord_i], Y[coord_i]])
    print(xypoints)
    xypoints = np.array(xypoints)
    tri = Delaunay(xypoints)
    indices = tri.simplices
    plt.triplot(xypoints[:,0], xypoints[:,1], tri.simplices)
    #plt.plot(xypoints[:,0], xypoints[:,1])

    #plt.scatter(vertices[:,0],vertices[:,1], marker = 'o')

    plt.show()
    return xypoints, indices 


def volumen_un_prisma(vertices2d, verticesZ):
	
	a, b, c = vertices2d
	altura = (verticesZ[0] + verticesZ[1] + verticesZ[2])/3
	restando1 =(b[0] - a[0])*(c[1] - a[1])
	restando2 = (c[0] - a[0])*(b[1] - a[1])

	area = abs(restando1 - restando2)/2
	volumen = altura*area
	return volumen


def volumen(X,Y,Z):
    points, vertices_indexs = tesselation(X,Y)
    altura = np.zeros_like(Z)
    for Z_i in range(len(Z)):
        if 6 - Z_i <= 0:
            altura[Z_i] = 0 
        else:
            altura[Z_i] = 6-Z[Z_i]
    altura = np.array(altura)

    volumen = 0
    for triangulo_i in range(len(vertices_indexs)):
        vertices_triangulo_i = [points[vertice_iesimo] for vertice_iesimo in vertices_indexs[triangulo_i]]
        alturas_triangulo_i = [altura[vertice_iesimo] for vertice_iesimo in vertices_indexs[triangulo_i]]

        vertices_triangulo_i = np.array(vertices_triangulo_i)
        alturas_triangulo_i = np.array(alturas_triangulo_i)
        volumen += volumen_un_prisma(vertices_triangulo_i, alturas_triangulo_i)
    print(altura)
    return volumen

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
    print(real_traj)
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

        XYZcoords = plot_measure(theta_angles, phi_angles, distances[0], MINDISTANCE,MAXDISTANCE, titlei = titles[i])
        [X, Y, Z] = plot_with_encoder(real_traj, distances[0], MINDISTANCE,MAXDISTANCE, titlei = titles[i])
        #XYZsplines[titles[i]] = [X,Y,Z]

    # curves[i][j] corresponds to the curve of the j-th point of the i-th iteration
