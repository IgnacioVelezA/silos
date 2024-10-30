import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider, TextBox
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

# Function to generate random points within the cube
def generate_points(num_points, cube_side):
    points = np.random.rand(num_points, 2)  # 2D array of points [x, y]
    x = points[:, 0] * cube_side
    y = points[:, 1] * cube_side
    z = np.zeros(num_points)
    return x, y, z

# Function to plot the resizing cube and highlight points within it
def plot_resizing_cube(width, depth, height):
    ax.cla()  # Clear the previous plot

    # Plot the dots within the specified width, depth, and height
    mask = (x <= width) & (y <= depth) & (z <= height)
    scat1 = ax.scatter(x[mask], y[mask], z[mask], c='blue', alpha=0.6, label="Points inside Cube")
    scat2 = ax.scatter(x[~mask], y[~mask], z[~mask], c='grey', alpha=0.3, label="Points outside Cube")

    # Cube vertices and faces
    vertices = np.array([[0, 0, 0], [width, 0, 0], [width, depth, 0], [0, depth, 0],
                         [0, 0, height], [width, 0, height], [width, depth, height], [0, depth, height]])

    faces = [[vertices[j] for j in [0, 1, 2, 3]], [vertices[j] for j in [4, 5, 6, 7]], 
             [vertices[j] for j in [0, 1, 5, 4]], [vertices[j] for j in [1, 2, 6, 5]],
             [vertices[j] for j in [2, 3, 7, 6]], [vertices[j] for j in [3, 0, 4, 7]]]
    
    ax.add_collection3d(Poly3DCollection(faces, facecolors='cyan', linewidths=1, edgecolors='r', alpha=0.15))
    ax.set_xlim(0, max(cube_side, width))
    ax.set_ylim(0, max(cube_side, depth))
    ax.set_zlim(0, max(height, 1))
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.legend(loc="upper right")

    # Update the count of points inside the cube
    points_inside = np.sum(mask)

    plt.draw()
    return scat1, scat2, points_inside

# Event handler to print coordinates of clicked points
def on_pick(event):
    if (event.artist != scat1)&(event.artist != scat2):
        print('ñe')
        return    
    ind = event.ind[0]  # Index of the picked point
    print(f"Clicked point coordinates: ({x[ind]:.2f}, {y[ind]:.2f}, {z[ind]:.2f})")

# Initialize the plot
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
num_points = 1000
cube_side = 5

# Generate the cube of dots
x, y, z = generate_points(num_points, cube_side)
initial_width = initial_depth = initial_height = 1

# Initial plot of the points and cube
scat1, scat2, initial_num_points_inside = plot_resizing_cube(initial_width, initial_depth, initial_height)

# Add height slider
axheight = plt.axes([0.25, 0.02, 0.65, 0.03], facecolor='lightgoldenrodyellow')
height_slider = Slider(axheight, 'Height', 0.1, cube_side, valinit=initial_height)

# Add width and depth text boxes side by side
axdepthbox = plt.axes([0.1, 0.25, 0.15, 0.05])  # Left TextBox for depth
depth_textbox = TextBox(axdepthbox, 'Depth', initial=str(initial_depth))

axwidthbox = plt.axes([0.3, 0.25, 0.15, 0.05])  # Right TextBox for width
width_textbox = TextBox(axwidthbox, 'Width', initial=str(initial_width))

# Add a textbox to display the count of points inside the cube
axcountbox = plt.axes([0.1, 0.35, 0.15, 0.05])
count_textbox = TextBox(axcountbox, 'Points Inside', initial="0")
count_textbox.set_active(False)  # Make the textbox read-only

# Update function to adjust plot based on inputs
def update(val):
    global scat1, scat2  # Update the scatter plot object
    width = float(width_textbox.text)
    depth = float(depth_textbox.text)
    height = height_slider.val
    scat1, scat2, points_inside = plot_resizing_cube(width, depth, height)
    count_textbox.set_val(str(points_inside))
    fig.canvas.mpl_connect("pick_event", on_pick)

    # Enable point picking on scatter plot
    scat1.set_picker(True)

    scat2.set_picker(True)

# Attach the update function to slider and text boxes
height_slider.on_changed(update)
width_textbox.on_submit(update)
depth_textbox.on_submit(update)

# Connect the event for printing point coordinates on click
fig.canvas.mpl_connect("pick_event", on_pick)

# Enable point picking on scatter plot
scat1.set_picker(True)

scat2.set_picker(True)
plt.show()
