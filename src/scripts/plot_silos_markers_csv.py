import pickle as pkl
import numpy as np
import argparse
import matplotlib.pyplot as plt
import os
import plotly.graph_objects as go
import scipy.interpolate
from scipy.spatial import Delaunay

from matplotlib.widgets import Slider, TextBox, Button, CheckButtons
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from shapely.geometry import point
from src.scripts import distanceFinder, readcsv

# Polyfit from https://stackoverflow.com/questions/7997152/python-3d-polynomial-surface-fit-order-dependent

angulo_zero = 0#30*np.pi/180
#angulo_zero = 30*np.pi/180



#////plot_measure==================================================================================
def plot_measure(theta_list, phi_list, distance_measurements, minAxis, maxAxis, titlei = False, plotornotplot = True):
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
        X[i] = np.sin(theta_list_rad[i]) * np.sin(phi_list_rad[i]) * (distance_centered[i])
        Y[i] = np.sin(theta_list_rad[i]) * np.cos(phi_list_rad[i]) * (distance_centered[i])
        Z[i] = -distance_centered[i]*np.cos(theta_list_rad[i])
        index[i] = f'punto {i}; elev: {theta_list[i]}; azi: {phi_list[i]}; rawdist = {distance_measurements[i]}'

        if distance_measurements[i] > 30:
           Z[i] = -35

        vertices[i] = np.array([X[i], Y[i], Z[i]])

    if plotornotplot:
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
    return [X,Y,Z]
#////END: plot_measure=============================================================================


#////plot_curve====================================================================================
def plot_curve(i, xs, thr, jump):
    curve = curves[i]
    distance, cleaned_curve, x_interpl= distanceFinder.distanceSplines(curve, thr ,MINDISTANCE,MAXDISTANCE, jump)
    z = np.cos(theta_angles[i]*np.pi/180)*distance
    title = f'Punto {i} threshold {thr}, salto de {jump}m'

    figc = plt.figure()
    plotcurve = plt.plot(xs, curve)
    plotcl = plt.plot(x_interpl, cleaned_curve)
    plt.vlines(distance,0,50, label=f'radial={distance}')
    plt.xlabel('distance[m]')
    plt.ylabel('Power')
    plt.title(f'{title}')
    plt.legend()
    plt.show()
    return cleaned_curve
#////END: plot_curve===============================================================================


def correct_real_traj(traj_measured, traj_commanded, LS_positions):
    offset_LS_azimutal = LS_positions[0]
    offset_LS_elev = LS_positions[1]
    real_traj_corr = []

    for punto_i in range(len(real_traj)):
        if traj_measured[punto_i][0] >= 5600:
            azimutal_encoder = traj_commanded[punto_i][0]
        else:
            azimutal_encoder = 92 - (offset_LS_azimutal - traj_measured[punto_i][0]) * 90/1024
            if azimutal_encoder > 180.0:
                azimutal_encoder = azimutal_encoder - 360.0
        if traj_measured[punto_i][1] >= 5600:
            elevation_encoder = traj_commanded[punto_i][1]
        else:
            elevation_encoder = 39 + (offset_LS_elev - traj_measured[punto_i][1]) * 90/1024
            if azimutal_encoder > 180.0:
                azimutal_encoder = azimutal_encoder - 360.0
        real_traj_corr.append((azimutal_encoder, elevation_encoder))
    return real_traj_corr


#////plot_with_encoder ===============================================================================
def plot_with_encoder(phi_list, traj_measured,traj_commanded, distance_measurements, minAxis, maxAxis, LS_positions, titlei = False):
    real_traj_corr = correct_real_traj(traj_measured, traj_commanded, LS_positions)
    print(real_traj_corr)
    theta_list_rad = [value[1] * np.pi/180 for value in real_traj_corr]
    #phi_list_rad = [value[0] * np.pi/180 for value in real_traj_corr]
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
        index[i] = f'punto {i}; elev: {real_traj_corr[i][1]}; azi: {real_traj_corr[i][0]}; rawdist = {distance_measurements[i]}'
        if distance_measurements[i] > 30:
           Z[i] = -35

    title = titlei + '\n Using encoders'
    # Configures the figure based on the chosen visualization
    fig2 = go.Figure()

    scatter = fig2.add_trace(go.Scatter3d(
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

    fig2.update_layout(scene=dict(
        xaxis=dict(title='Eje X'),
        yaxis=dict(title='Eje Y'),
        zaxis=dict(title='Eje Z', range = [-maxAxis,minAxis]),
        ),title = title)
        
    #fig.colorbar(scatter, shrink=0.5, aspect=5, label = "Distance [m]")

    fig2.show()

    while True:
        mark = input('Indice (just enter to break):')
        if mark:
            #if mark == x:
            indxMark = int(mark)
            fig2 = go.Figure()
            scatter = fig2.add_trace(go.Scatter3d(
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
            scatter = fig2.add_trace(go.Scatter3d(
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

            fig2.show()
        else:
            break
    return [X, Y, Z], real_traj_corr


def tesselation(xypoints):
	tri = Delaunay(xypoints)
	indices = tri.simplices
	return indices

def volumen_un_prisma(vertices2d, verticesZ):
	
	a, b, c = vertices2d
	altura = (verticesZ[0] + verticesZ[1] + verticesZ[2])/3
	restando1 =(b[0] - a[0])*(c[1] - a[1])
	restando2 = (c[0] - a[0])*(b[1] - a[1])

	area = abs(restando1 - restando2)/2
	volumen = altura*area
	return volumen

def zcalculator(zvector, height_limit, bottom_limit, floor):
    newzvector = np.zeros_like(zvector)
    for zi in range(len(zvector)):
        newz = min(zvector[zi], height_limit) - floor
        newzvector[zi] = newz-max(bottom_limit,0)
    return newzvector


def full_volume(pointsxy,z):
    if not(pointsxy.any()):
        return 0.0

    vertices_indexs = tesselation(pointsxy)
    volumen = 0
    for triangulo_i in range(len(vertices_indexs)):
        vertices_triangulo_i = [pointsxy[vertice_iesimo] for vertice_iesimo in vertices_indexs[triangulo_i]]
        alturas_triangulo_i = [z[vertice_iesimo] for vertice_iesimo in vertices_indexs[triangulo_i]]

        vertices_triangulo_i = np.array(vertices_triangulo_i)
        alturas_triangulo_i = np.array(alturas_triangulo_i)
        volumen += volumen_un_prisma(vertices_triangulo_i, alturas_triangulo_i)

    return round(volumen,4)

def plot_resizing_cube(x,y,z,xside, yside, zside, limits):
    ax.cla()  # Clear the previous plot
    x0,x1 = xside
    y0,y1 = yside
    bottom, height = zside
    limitx, limity, limitz = limits
    # Plot the dots within the specified width, depth, and height
    mask = (x >= x0) & (x <= x1) & (y >= y0) & (y <= y1) & (z >= bottom) & (z <= height)
    scat1 = ax.scatter(x[mask], y[mask], z[mask], c='blue', alpha=0.9, label="Points inside Cube")
    scat2 = ax.scatter(x[~mask], y[~mask], z[~mask], c=z[~mask], alpha=1, cmap="rainbow_r")

    # Cube vertices and faces
    vertices = np.array([[x0, y0, bottom], [x1, y0, bottom], [x1, y1, bottom], [x0, y1, bottom],
                         [x0, y0, height], [x1, y0, height], [x1, y1, height], [x0, y1, height]])

    faces = [[vertices[j] for j in [0, 1, 2, 3]], [vertices[j] for j in [4, 5, 6, 7]], 
             [vertices[j] for j in [0, 1, 5, 4]], [vertices[j] for j in [1, 2, 6, 5]],
             [vertices[j] for j in [2, 3, 7, 6]], [vertices[j] for j in [3, 0, 4, 7]]]
    
    ax.add_collection3d(Poly3DCollection(faces, facecolors='cyan', linewidths=1, edgecolors='r', alpha=0.15))
    ax.set_xlim(limitx[0], limitx[1])
    ax.set_ylim(limity[0], limity[1])
    ax.set_zlim(limitz[0], limitz[1])
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.legend(loc="upper right")

    num_points_inside = np.sum(mask)
    plt.draw()
    return scat1, scat2, num_points_inside,mask


# Event handler to print coordinats of clicked points
def on_pick(event):
    if (event.artist != scat1)&(event.artist != scat2):
        print('ñe')
        return    
    ind = event.ind[0]  # Index of the picked point
    print(f"Clicked point coordinates: ({x[ind]:.2f}, {y[ind]:.2f}, {z[ind]:.2f}, ind:{ind})")

def on_button_click(event, point2plot):
    plot_curve(point2plot, xs,0,0)

if __name__=='__main__':
    parser = argparse.ArgumentParser(description='plots a silos measurement'+
                                                 ' from a csv file')
    parser.add_argument('filename', help='name of the file')
    parser.add_argument('-min', '--MINDISTANCE', default = 0, type = int)
    parser.add_argument('-max', '--MAXDISTANCE', default = 30, type = int)
    parser.add_argument('-thr', '--threshold', default = 15, type = int)
    args = parser.parse_args()

    filename = args.filename

    if filename[-4:] == '.csv':
        name = filename[:-14]
        date = filename[-14:-4]
        fileDir = 'measurements/' + date + '/'
        fileDir = fileDir + filename
    else:
        name = filename[:-10]
        date = filename[-10:]
        fileDir = 'measurements/' + date +'/'
        fileDir = fileDir + filename + '.csv'

    # saving arguments to variables
    threshold = args.threshold
    MINDISTANCE = args.MINDISTANCE
    MAXDISTANCE = args.MAXDISTANCE

    xs = np.array(np.arange(start=MINDISTANCE, stop=MAXDISTANCE, step=(MAXDISTANCE-MINDISTANCE)/128))

    traj_angle, real_traj, curves, LS_positions = readcsv.read_csv_measurements(fileDir)

    #print(real_traj)
    # saving target angles
    #print(curves)
    n_points = len(curves) #<------------cambiar para determinar n de puntos

    phi_angles = np.zeros(n_points)
    theta_angles = np.zeros(n_points)

    for i in range(n_points):
        phi_angles[i] = traj_angle[i][0]
        theta_angles[i] = traj_angle[i][1]

    distances = np.zeros((1,n_points)) # [iter][distance]


    for j in range(n_points):
        distances[0][j], _, _ = distanceFinder.distanceSplines(curves[j], threshold,MINDISTANCE,MAXDISTANCE, 5, 1)

        # distances[i] corresponds to the measured distances for the i-th iteration
        # plots using the visualization set as argument

    filename = filename + ' con Umbral ' + str(threshold)
    XYZcoords = plot_measure(theta_angles, phi_angles, distances[0], MINDISTANCE,MAXDISTANCE, titlei = filename)
    XYZ_real, real_traj_corr = plot_with_encoder(phi_angles, real_traj, traj_angle, distances[0], MINDISTANCE,MAXDISTANCE, LS_positions,titlei =filename)
    #XYZsplines[titles[i]] = [X,Y,Z]
    #print(volumen(XYZcoords[0], XYZcoords[1], XYZcoords[2]))
    # curves[i][j] corresponds to the curve of the j-th point of the i-th iteration

    ### ----------------------------------------------
    fig3 = plt.figure()
    ax = fig3.add_subplot(111, projection='3d')

    x,y,z=plot_measure(theta_angles, phi_angles, distances[0], MINDISTANCE,MAXDISTANCE, titlei = filename, plotornotplot=False)#XYZcoords[0], XYZcoords[1], XYZcoords[2]

    initial_X1 = initial_Y1 =  round(max(max(x),max(y)),4)
    initial_height = 0
    initial_X0 = initial_Y0 = initial_bottom = round(min(min(x),min(y)),4)
    initial_bottom = min(z)
    plot_side= round(max(max(x),max(y)) - min(min(x),min(y)),4)

    initial_xside = [initial_X0, initial_X1]
    initial_yside = [initial_Y0, initial_Y1]
    initial_zside = [initial_bottom, initial_height]
    limits = [initial_xside, initial_yside, initial_zside]
    # Initial plot of the points and cube
    scat1, scat2, initial_num_points_inside, mask = plot_resizing_cube(x,y,z,initial_xside, initial_yside, initial_zside, limits)
    initial_points_inside = np.zeros([initial_num_points_inside,2])
    initial_points_inside[:,0] = x[mask]
    initial_points_inside[:,1] = y[mask]
    initial_z_inside = z[mask]


    initial_z_inside_adjust = zcalculator(initial_z_inside, initial_height, initial_bottom, initial_bottom)
    initial_volume_inside = full_volume(initial_points_inside,initial_z_inside_adjust)

    initial_point2plot = 0

    # Add height slider
    axheight = plt.axes([0.25, 0.09, 0.65, 0.03], facecolor='lightgoldenrodyellow')
    height_slider = Slider(axheight, 'Height', initial_bottom+0.5, 0, valinit=initial_height)
    # Add bottom slider
    axbottom = plt.axes([0.25, 0.06, 0.65, 0.03], facecolor='lightgoldenrodyellow')
    bottom_slider = Slider(axbottom, 'bottom', initial_bottom-0.5, -0.5, valinit=initial_bottom, slidermax = height_slider)

    axumbral = plt.axes([0.25, 0.03, 0.65, 0.03], facecolor='lightgoldenrodyellow')
    umbral_slider = Slider(axumbral, 'Umbral', 0, 30,valinit=0)

    # Add width and depth text boxes
    axX0box = plt.axes([0.1, 0.3, 0.05, 0.05])
    X0_textbox = TextBox(axX0box, 'X0', initial=str(initial_X0))

    # Add width and depth text boxes
    axX1box = plt.axes([0.2, 0.3, 0.05, 0.05])
    X1_textbox = TextBox(axX1box, 'X1', initial=str(initial_X1))

    # Add width and depth text boxes
    axY0box = plt.axes([0.1, 0.4, 0.05, 0.05])
    Y0_textbox = TextBox(axY0box, 'Y0', initial=str(initial_Y0))

    # Add width and depth text boxes
    axY1box = plt.axes([0.2, 0.4, 0.05, 0.05])
    Y1_textbox = TextBox(axY1box, 'Y1', initial=str(initial_Y1))

    # Add a textbox to display the count of points inside the cube
    axcountbox = plt.axes([0.1, 0.5, 0.1, 0.05])
    count_textbox = TextBox(axcountbox, 'Points Inside', initial=str(initial_num_points_inside))
    count_textbox.set_active(False)  # Make the textbox read-only

    # Add a textbox to display the count of points inside the cube
    axvolumebox = plt.axes([0.1, 0.6, 0.1, 0.05])
    volume_textbox = TextBox(axvolumebox, 'volume Inside', initial=str(initial_volume_inside))
    volume_textbox.set_active(False)  # Make the textbox read-only

    # Add width and depth text boxes
    axpointbox = plt.axes([0.8, 0.6, 0.05, 0.05], label = 'aaaa')
    point_textbox = TextBox(axpointbox, 'Point selected', initial=str(initial_point2plot))
    point2plot = initial_point2plot


    # Add a textbox to display the count of points inside the cube
    axcubebuttonbox = plt.axes([0.1, 0.7, 0.05, 0.05], facecolor="grey")
    volumen_checkbutton = CheckButtons(axcubebuttonbox, ' ', [False])
    #check_text = plt.text(0.1, 0.75, 'Calculate Volumen')

    # Add a textbox to display the count of points inside the cube
    axbuttonbox = plt.axes([0.8, 0.4, 0.1, 0.05])
    plot_button = Button(axbuttonbox, 'Plot Curve')
    # Update function to adjust plot based on inputs
    DoIntegrate = False

    def update(val):
        global scat1, scat2, point2plot 
        X0 = float(X0_textbox.text)
        X1 = float(X1_textbox.text)
        Y0 = float(Y0_textbox.text)
        Y1 = float(Y1_textbox.text)
        point2plot = int(point_textbox.text)
        xside = [X0,X1]
        yside = [Y0, Y1]
        height = height_slider.val
        bottom = bottom_slider.val
        umbral = umbral_slider.val

        zside = [bottom, height]
        height_slider.slidermin = bottom_slider
        bottom_slider.slidermax = height_slider

        for j in range(n_points):
            distances[0][j], _, _ = distanceFinder.distanceSplines(curves[j], umbral,MINDISTANCE,MAXDISTANCE, 5, 1)

        x,y,z = plot_measure(theta_angles, phi_angles, distances[0], MINDISTANCE,MAXDISTANCE, titlei = filename, plotornotplot=False)#XYZcoords[0], XYZcoords[1], XYZcoords[2]

        #if DoIntegrate:
        scat1, scat2, num_points_inside, mask= plot_resizing_cube(x,y,z,xside, yside,zside, limits)
        fig3.canvas.mpl_connect("pick_event", on_pick)
        scat1.set_picker(True)
        scat2.set_picker(True)

        points_inside = np.zeros([num_points_inside, 2])
        points_inside[:,0] = x[mask]
        points_inside[:,1] = y[mask]
        z_updated = zcalculator(z[mask], height, bottom, initial_bottom)
        volume =full_volume(points_inside,z_updated)

        count_textbox.set_val(str(num_points_inside))
        volume_textbox.set_val(str(volume))

    # Attach the update function to slider and text boxes
    height_slider.on_changed(update)
    X0_textbox.on_submit(update)
    X1_textbox.on_submit(update)
    Y0_textbox.on_submit(update)
    Y1_textbox.on_submit(update)
    bottom_slider.on_changed(update)
    umbral_slider.on_changed(update)
    point_textbox.on_submit(update)
    volumen_checkbutton.on_clicked(lambda x: print('ok'))
    plot_button.on_clicked(lambda x: on_button_click(x, point2plot))
    # Connect the event for printing point coordinates on click
    fig3.canvas.mpl_connect("pick_event", on_pick)

    # Enable point picking on scatter plot
    scat1.set_picker(True)

    scat2.set_picker(True)
    plt.show()