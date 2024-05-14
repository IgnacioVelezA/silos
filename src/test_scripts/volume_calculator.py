import numpy as np
from scipy.interpolate import griddata

# Ejemplo de datos de alturas (x, y, altura)
# Aquí debes reemplazar estos datos con los tuyos
data = np.array([
    [1, 1, 10],
    [2, 1, 20],
    [1, 2, 15],
    [2, 2, 25]
])

# Extraer coordenadas x, y y alturas
x = data[:, 0]
y = data[:, 1]
z = data[:, 2]

# Definir la cuadrícula para interpolar
xi = np.linspace(min(x), max(x), 100)
yi = np.linspace(min(y), max(y), 100)
xi, yi = np.meshgrid(xi, yi)

# Interpolar alturas en la cuadrícula
zi = griddata((x, y), z, (xi, yi), method='linear')

# Calcular el volumen total (sumando todos los valores de altura)
volumen_total = np.sum(zi)

print("Volumen total:", volumen_total)