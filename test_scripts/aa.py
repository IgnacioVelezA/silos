import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider, TextBox
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from scipy.spatial import Delaunay

def zcalculator(zvector, height_limit, bottom_limit):
    newzvector = np.zeros_like(zvector)
    for zi in range(len(zvector)):
        newz = min(zvector[zi], height_limit)
        newzvector[zi] = newz-max(bottom_limit,0)
    return newzvector


def full_volume(points,z):
    if not(points.any()):
        print(0.0)
        return 0.0

    vertices_indexs = tesselation(points)
    volumen = 0
    for triangulo_i in range(len(vertices_indexs)):
        vertices_triangulo_i = [points[vertice_iesimo] for vertice_iesimo in vertices_indexs[triangulo_i]]
        alturas_triangulo_i = [z[vertice_iesimo] for vertice_iesimo in vertices_indexs[triangulo_i]]

        vertices_triangulo_i = np.array(vertices_triangulo_i)
        alturas_triangulo_i = np.array(alturas_triangulo_i)
        volumen += volumen_un_prisma(vertices_triangulo_i, alturas_triangulo_i)

    print(volumen)
    return round(volumen,4)


def volumen_un_prisma(vertices2d, verticesZ):
	a, b, c = vertices2d
	altura = (verticesZ[0] + verticesZ[1] + verticesZ[2])/3
	restando1 = (b[0] - a[0])*(c[1] - a[1])
	restando2 = (c[0] - a[0])*(b[1] - a[1])

	area = abs(restando1 - restando2)/2
	volumen = altura*area
	return volumen


def tesselation(xypoints):
	tri = Delaunay(xypoints)
	indices = tri.simplices
	return indices


# Function to generate random points for the cube of dots
def generate_points(num_points):
    cube_shape = (num_points/5,num_points/5)
    # Generate random x and y coordinates and store them in a 2D numpy array
    points = np.random.rand(num_points, 2)  # Each row is a point [x, y]

    # Split the array into x and y for plotting
    x = points[:, 0]
    y = points[:, 1]
    z = np.zeros(num_points)


    for zi in range(len(z)):
        xorigin = (num_points/2-cube_shape[0])/num_points
        xend = (num_points/2+cube_shape[0])/num_points
        yorigin = (num_points/2-cube_shape[1])/num_points
        yend = (num_points/2+cube_shape[1])/num_points
        if x[zi] <= xend and x[zi] >= xorigin:
            if y[zi] <= yend and y[zi] >= yorigin:
                z[zi] = 1

    return points, z

# Function to plot the resizing cube and highlight points within it
def plot_resizing_cube(x,y,z,xside, yside, zside):
    ax.cla()  # Clear the previous plot
    x0,x1 = xside
    y0,y1 = yside
    bottom, height = zside

    # Plot the dots within the specified width, depth, and height
    mask = (x >= x0) & (x <= x1) & (y >= y0) & (y <= y1) & (z >= bottom) & (z <= height)
    ax.scatter(x[mask], y[mask], z[mask], c='blue', alpha=0.6, label="Points inside Cube")
    ax.scatter(x[~mask], y[~mask], z[~mask], c='grey', alpha=0.3, label="Points outside Cube")

    # Cube vertices and faces
    vertices = np.array([[x0, y0, bottom], [x1, y0, bottom], [x1, y1, bottom], [x0, y1, bottom],
                         [x0, y0, height], [x1, y0, height], [x1, y1, height], [x0, y1, height]])

    faces = [[vertices[j] for j in [0, 1, 2, 3]], [vertices[j] for j in [4, 5, 6, 7]], 
             [vertices[j] for j in [0, 1, 5, 4]], [vertices[j] for j in [1, 2, 6, 5]],
             [vertices[j] for j in [2, 3, 7, 6]], [vertices[j] for j in [3, 0, 4, 7]]]
    
    ax.add_collection3d(Poly3DCollection(faces, facecolors='cyan', linewidths=1, edgecolors='r', alpha=0.15))
    ax.set_xlim(0, max(plot_side, x1))
    ax.set_ylim(0, max(plot_side, y1))
    ax.set_zlim(-0.2, max(height, 1))
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.legend(loc="upper right")

    num_points_inside = np.sum(mask)
    plt.draw()
    return num_points_inside,mask
    
# Initialize the plot
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
num_points = 1000
plot_side= 1


# Generate the cube of dots
points, z = generate_points(num_points)
x = points[:, 0]
y = points[:, 1]
initial_X1 = initial_Y1 = initial_height = 1

initial_X0 = initial_Y0 = initial_bottom = 0

initial_xside = [initial_X0, initial_X1]
initial_yside = [initial_Y0, initial_Y1]
initial_zside = [initial_bottom, initial_height]
# Initial plot of the points and cube
initial_num_points_inside, mask = plot_resizing_cube(x,y,z,initial_xside, initial_yside, initial_zside)
initial_points_inside = np.zeros([initial_num_points_inside,2])
initial_points_inside[:,0] = x[mask]
initial_points_inside[:,1] = y[mask]
initial_z_inside = z[mask]

initial_volume_inside = full_volume(initial_points_inside,initial_z_inside)
# Add height slider
axheight = plt.axes([0.25, 0.08, 0.65, 0.03], facecolor='lightgoldenrodyellow')
height_slider = Slider(axheight, 'Height', 0.1, plot_side, valinit=initial_height)
# Add bottom slider
axbottom = plt.axes([0.25, 0.04, 0.65, 0.03], facecolor='lightgoldenrodyellow')
bottom_slider = Slider(axbottom, 'bottom', -0.1, plot_side, valinit=initial_bottom, slidermax = height_slider)

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

# Update function to adjust plot based on inputs
def update(val):
    X0 = float(X0_textbox.text)
    X1 = float(X1_textbox.text)
    Y0 = float(Y0_textbox.text)
    Y1 = float(Y1_textbox.text)
    xside = [X0,X1]
    yside = [Y0, Y1]
    height = height_slider.val
    bottom = bottom_slider.val
    zside = [bottom, height]
    height_slider.slidermin = bottom_slider
    bottom_slider.slidermax = height_slider
    num_points_inside, mask= plot_resizing_cube(x,y,z,xside, yside,zside)

    points_inside = np.zeros([num_points_inside, 2])
    points_inside[:,0] = x[mask]
    points_inside[:,1] = y[mask]
    z_updated = zcalculator(z[mask], height, bottom)
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

plt.show()
