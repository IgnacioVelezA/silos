import csv
import numpy as np

def  tuple2float(strtuple):
        # estructura de strtuple '([signo]wx,[signo]yz)'
        strtuple = strtuple.replace('(','')
        strtuple = strtuple.replace(')','')
        coords = strtuple.split(',')
        az = float(coords[0])
        el = float(coords[1]) 
        return (az,el)


def  tuple2int(strtuple):
        # estructura de strtuple '([signo]wx,[signo]yz)'
        strtuple = strtuple.replace('(','')
        strtuple = strtuple.replace(')','')
        coords = strtuple.split(',')
        az = int(coords[0])
        el = int(coords[1]) 
        return (az,el)
 

def read_csv_measurements(filename):
    az_encoder_positions = []
    el_encoder_positions = []
    az_positions = []
    el_positions = []
    curves = []

    with open(filename, 'r') as file:
        reader = csv.reader(file)
        next(reader)  # Saltar la fila de encabezado

        for row in reader:
            az_position, el_position = tuple2float(row[1])           
            az_positions.append(az_position)
            el_positions.append(el_position)
            az_encoder_position, el_encoder_position = tuple2int(row[2])            
            az_encoder_positions.append(az_encoder_position)
            el_encoder_positions.append(el_encoder_position)
            # Procesar la cadena de la curva para agregar comas entre los números
            curve_str = row[3].replace('  ', ' ')  # Reemplaza dobles espacios por un solo espacio
            curve_list = [int(num) for num in curve_str.strip('[]').split()]  # Convierte la cadena en una lista de números
                

            curves.append(curve_list)

    # Convertir listas a arrays de NumPy
    az_positions_np = np.array(az_positions)
    el_positions_np = np.array(el_positions)
    az_encoder_positions_np = np.array(az_encoder_positions)
    el_encoder_positions_np = np.array(el_encoder_positions)
    curves_np = np.array(curves)

    # Crear un array de NumPy para la trayectoria real (con AZ y EL)
    trajectory_np = np.column_stack((az_positions_np, el_positions_np))
    real_trajectory_np = np.column_stack((az_encoder_positions_np, el_encoder_positions_np))

    return trajectory_np, real_trajectory_np, curves_np