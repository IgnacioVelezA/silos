import numpy as np

def convertGPT(measure_bits, LS_bits, LS_pos_degrees=-17):
    # Convertir bits a grados
    measure_degrees = measure_bits * 90 / 1024
    LS_degrees = LS_bits * 90 / 1024
    
    # Calcular la posición medida desde el LS
    measure_from_LS = measure_degrees - LS_degrees
    
    # Ajustar con la posición del LS
    position_corrected = measure_from_LS + LS_pos_degrees
    
    # Normalizar el ángulo en el rango -180 a 180
    angle = position_corrected % 360
    if angle > 180:
        angle -= 360
    elif angle < -180:
        angle += 360
    
    return 90-angle


