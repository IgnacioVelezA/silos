
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
 

az_encoder_positions = []
el_encoder_positions = []
az_positions = []
el_positions = []
curves = []

filename = 'prueba_piso_2_r0:3.0_rmax:30.0_2024-09-02.csv'
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

filecopia='prueba_piso_2_r0:3.0_rmax:30.0_copia_2024-09-02.csv'
with open(filecopia, mode='w', newline='') as filecsv:
        writer = csv.writer(filecsv)
        writer.writerow(['Measure Time', 'Commanded trajectory', 'Real trajectory', 'Curve'])
        filecsv.close()

with open(filecopia, mode='a', newline='') as filecsv:
        for i in range(len(trajectory_np)):
                pos_commanded = trajectory_np[i]
                if pos_commanded[0] >= 0.0:
                        ideal_az_i = pos_commanded[0]*1024/90
                else:
                        ideal_az_i = (360+pos_commanded[0])*1024/90
                
                if pos_commanded[1] >= 0.0:
                        ideal_el_i = pos_commanded[1]*1024/90
                else:
                        ideal_el_i = (360+pos_commanded[1])*1024/90
                traj_i = (trajectory_np[i][0], trajectory_np[i][1])
                ideal_position_i = (int(ideal_az_i), int(ideal_el_i))
                writer = csv.writer(filecsv)
                writer.writerow([i, traj_i, ideal_position_i, curves_np[i]])

        filecsv.close()