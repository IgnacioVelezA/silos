import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial import Delaunay

def tesselation(xypoints, triangulo_n):
	tri = Delaunay(xypoints)
	indices = tri.simplices
	altura = np.ones(len(xypoints))*-1
	plt.triplot(xypoints[:,0], xypoints[:,1], tri.simplices)
	vertices_indx = indices[triangulo_n]
	vertices = [xypoints[i] for i in vertices_indx]
	vertices = np.array(vertices)
	plt.plot(xypoints[:,0], xypoints[:,1])

	#plt.scatter(vertices[:,0],vertices[:,1], marker = 'o')

	#plt.show()
	return vertices
# Number of random points
num_points = 500
cube_shape = (num_points/5,num_points/5)
# Generate random x and y coordinates and store them in a 2D numpy array
points = np.random.rand(num_points, 2)  # Each row is a point [x, y]

# Split the array into x and y for plotting
x = points[:, 0]
y = points[:, 1]
z = np.ones(num_points)

for zi in range(len(z)):
	xorigin = (num_points/2-cube_shape[0])/num_points
	xend = (num_points/2+cube_shape[0])/num_points
	yorigin = (num_points/2-cube_shape[1])/num_points
	yend = (num_points/2+cube_shape[1])/num_points
	if x[zi] <= xend and x[zi] >= xorigin:
		if y[zi] <= yend and y[zi] >= yorigin:
			z[zi] = 2 

	z[zi] = z[zi] + np.random.normal(0,0.1,1)
# Create the plot
fig = plt.figure()

ax = fig.add_subplot(1,1,1, projection='3d')
#ax.set_box_aspect([1,1,1])
ax.scatter(x, y,z, c=z, cmap="gist_rainbow",alpha=1)
#ax.title('Random Point Cloud')
tesselation(points,3)
ax.set_ylabel('y axis(Vertical)')
ax.set_xlabel('x axis (Horizontal)')
ax.grid(True)
plt.show()


