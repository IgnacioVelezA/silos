import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial import Delaunay

def volumen_un_prisma(vertices2d, verticesZ):
	
	a, b, c = vertices2d
	altura = (verticesZ[0] + verticesZ[1] + verticesZ[2])/3
	restando1 = abs(b[0] - a[0])*abs(c[1] - a[1])
	restando2 = abs(c[0] - a[0])*abs(b[1] - a[1])

	area = abs(restando1 - restando2)/2
	volumen = altura*area
	return volumen


def tesselation(xypoints):
	tri = Delaunay(xypoints)
	indices = tri.simplices
	#plt.triplot(xypoints[:,0], xypoints[:,1], tri.simplices)
	#plt.plot(xypoints[:,0], xypoints[:,1])

	#plt.scatter(vertices[:,0],vertices[:,1], marker = 'o')

	#plt.show()
	return indices


# Number of random points
num_points = 1000000
cube_shape = (num_points,num_points)
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

	#z[zi] = z[zi] + np.random.normal(0,0.1,1)
# Create the plot
#fig = plt.figure()

#ax = fig.add_subplot(1,1,1, projection='3d')
#ax.set_box_aspect([1,1,1])
#ax.scatter(x, y,z, c=z, cmap="gist_rainbow",alpha=1)
#ax.title('Random Point Cloud')
vertices_indexs = tesselation(points)

volumen = 0
for triangulo_i in range(len(vertices_indexs)):
	vertices_triangulo_i = [points[vertice_iesimo] for vertice_iesimo in vertices_indexs[triangulo_i]]
	alturas_triangulo_i = [z[vertice_iesimo] for vertice_iesimo in vertices_indexs[triangulo_i]]

	vertices_triangulo_i = np.array(vertices_triangulo_i)
	alturas_triangulo_i = np.array(alturas_triangulo_i)
	volumen += volumen_un_prisma(vertices_triangulo_i, alturas_triangulo_i)

print(volumen)
# ax.set_ylabel('y axis(Vertical)')
# ax.set_xlabel('x axis (Horizontal)')
# ax.grid(True)
# plt.show()


