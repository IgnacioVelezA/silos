# This is the main script for silos measurement. Consists of 13 different points.

from src.radar_communication import serial_radar
from src.stepper_motor import serial_arduino
from src.stepper_motor import stepper_motor
import RPi.GPIO as GPIO
import time
import math

from datetime import datetime, timedelta, date
import pickle as pkl
import argparse
from src.trajectory import hex_trajectory

MOTOR_STATUS_POLLING_TIME = 0.1     # in seconds

WAIT_TIME_AFTER_MEASUREMENT = 0.1   # in seconds
LOWEST_EL_ANGLE = 30    # Angle between zenith and lowest el point to measure

measured_distances = []
real_trajectory = []


def point_and_measure(angles, measure_wait_time):
    az_motor.move(angles[0])
    el_motor.move(angles[1])
    
    # Wait for motors to reach target angle
    az_motor.wait_for_target_angle(angles[0])
    el_motor.wait_for_target_angle(angles[1])
    measure_time = datetime.utcnow()

    az_motor.set_current_angle()
    el_motor.set_current_angle()

    # Waits for the radar curve to refresh
    print("Arrived to position, waiting " + str(measure_wait_time) + "s for radar curve to refresh")
    time.sleep(measure_wait_time)

    curve = serial_radar.get_curve()

    print("Measured distance at AZ = " + str(angles[0])
        +" and EL = " + str(angles[1]))
    
    return measure_time, curve     #distance, curve)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='moves the radar to its 13 '+
                                                 'positions and logs the info')
    
    parser.add_argument('filename', help='name of the file')
    parser.add_argument('arduino_usb_port', help='arduino usb port for serial comm, tipically /dev/ttyUSB1')
    parser.add_argument('-r0','--radio_0', help='Radars HPBW')
    parser.add_argument('-rmax','--radio_max', help='Radio of circular surface to map')
    parser.add_argument('radar_usb_port', help='radar usb port for serial comm, tipically /dev/ttyUSB0')

    #parser.add_argument('-nt','--number_of_trajectories', help ='number of trajectories to execute \n'+
    #                                                   '1 by default, 0 means infinite')
    parser.add_argument('-ar','--angle_repetition', help ='number of repetitions per angle, 1 by default')

    parser.add_argument('-rwait','--radar_wait_time', type = int, default = 10, help ='radar wait time, in seconds, before taking the curve')

    
    args = parser.parse_args()

     # Defining points to measure
    try:
        RADIO_BEAM = float(args.radio_0)
    except TypeError:
        RADIO_BEAM = 3 
    
    try:
        RADIO_SURFACE = float(args.radio_max)
    except TypeError:
        RADIO_SURFACE = 30
        
    try:
        angle_rep = int(args.angle_repetition)
    except TypeError:
        angle_rep = 1

    # Defining wait time for radar
    radar_measure_wait_time = args.radar_wait_time

    traj = hex_trajectory.hex_trajectory(RADIO_BEAM, RADIO_SURFACE)


    time.sleep(1)
    todays_date = date.today()      

    # file name ----------------------------------------------------------
    nameSD = 'measurements/'
    #nameUSB = '/media/silos/15FC-A7CE/measurements/'
    
    if args.filename[-4:] == '.pkl':
        suffix = args.filename[:-4]+ '_'+ str(todays_date) + '.pkl'
        nameSD = nameSD + suffix
        #nameUSB = nameUSB + suffix
    else:
        suffix = args.filename+ '_' + str(todays_date) + '.pkl'
        nameSD = nameSD + suffix
        #nameUSB = nameUSB + suffix
    
    # opening files ------------------------------------------------------
    try :
        # if files exist, just append data to it
        fileSD = open(nameSD,'rb')
        fileSD.close()
        # fileUSB = open(nameUSB, 'rb')
        # fileUSB.close()

    except FileNotFoundError:
        #if file does not exist initialize it
        config_dict = {'params': (RADIO_BEAM, RADIO_SURFACE),
                       'traj' : traj}

        fileSD = open(nameSD,'wb')
        pkl.dump(config_dict,fileSD)
        fileSD.close()

        # fileUSB = open(nameUSB, 'wb')
        # pkl.dump(config_dict,fileUSB)
        # fileUSB.close()

    # Initializing radar serial comm
    serial_radar.open_serial(args.radar_usb_port)
    if serial_radar.is_serial_open():
        print("Serial radar port open succesfully")
    else:
        print("Could not open serial radar port, FATAL ERROR")

    # Initializing radar motors, setting PID and limit switches
    GPIO.cleanup()
    serial_arduino.open_serial(args.arduino_usb_port)
    if serial_arduino.is_serial_open():
        print("Serial arduino port open succesfully")
    else:
        print("Could not open arduino radar port, FATAL ERROR")

    az_motor = stepper_motor.stepper_motor(id = 1,speed=0, max_speed=400, acceleration=600)
    az_motor.add_limit_switch(12,-90) #12
    az_motor.add_limit_switch(22,90) #24
    time.sleep(1)

    el_motor = stepper_motor.stepper_motor(id = 2, speed=0, max_speed=400, acceleration=600)
    el_motor.add_limit_switch(18,-44) #18
    el_motor.add_limit_switch(23,53) #23
    time.sleep(1)

    # Initializing motors position using limit switches

    az_motor.initialization(dir=1,speed=0, max_speed=400, acceleration=600)
    while not az_motor.is_initialized:
        time.sleep(MOTOR_STATUS_POLLING_TIME)
    az_motor.offset_angle_encoder = az_motor.read_encoder() #por ahora queda en 0 en la posición del ls reached
    az_motor.move(0)

    el_motor.initialization(dir=1,speed=0, max_speed=400, acceleration=600) 
    while not el_motor.is_initialized:
        time.sleep(MOTOR_STATUS_POLLING_TIME)
    el_motor.offset_angle_encoder = el_motor.read_encoder() #por ahora queda en 0 en la posición del ls reached
    el_motor.move(0)
    
    print("Both motors are initialized")
    
    data_counter = 0
    
    # Pointing and measuring the corresponding angles
    #trayectoria harcodeada
    traj =[(0, 0), (90, 5.0), (30.0, 5.0), (-30.0, 5.0), (-90, 5.0), (25.0, -10.0), (-35.0, -10.0), (90, 10.0), (61.12, 8.5652), (30.0, 10.0), (1.12, 8.5652), (-30.0, 10.0), (-58.88, 8.5652), (-90, 10.0), (56.12, -13.5653), (25.0, -15.0), (-4.12, -13.5652), (-35.0, -15.0), (-63.88, -13.5652), (90, 15.0), (71.69, 13.1667), (50.4, 12.9789), (30.0, 15.0), (11.69, 13.1667), (-9.6, 12.9789), (-30.0, 15.0), (-48.31, 13.1667), (-69.6, 12.9789), (-90, 15.0), (66.69, -18.1667), (45.4, -17.9789), (25.0, -20.0), (6.69, -18.1667), (-14.6, -17.9789), (-35.0, -20.0), (-53.31, -18.1667), (-79.6, -17.9789), (90, 20.0), (76.7, 17.9823), (61.12, 17.1304), (45.21, 17.6143), (30.0, 20.0), (16.7, 17.9823), (1.12, 17.1304), (-14.79, 17.6143), (-30.0, 20.0), (-43.3, 17.9823), (-58.88, 17.1304), (-74.79, 17.6143), (-90, 20.0), (71.7, -22.9823), (56.12, -22.1304), (40.21, -22.6144), (25.0, -25.0), (11.7, -22.9823), (-4.12, -22.1304), (-19.79, -22.6144), (-35.0, -25.0), (-48.3, -22.9823), (-63.88, -22.1304), (-79.79, -22.6143), (90, 25.0), (79.58, 22.8771), (67.53, 21.6437), (54.66, 21.4538), (42.19, 22.3339), (30.0, 25.0), (19.58, 22.8771), (7.53, 21.6438), (-5.34, 21.4538), (-17.81, 22.3339), (-30.0, 25.0), (-40.42, 22.8771), (-52.47, 21.6437), (-65.34, 21.4538), (-77.81, 22.3339), (-90, 25.0), (74.58, -27.8771), (62.53, -26.6438), (49.66, -26.4538), (32.19, -27.3339), (20.0, -30.0), (9.58, -27.8771), (-3.53, -26.6438), (-15.34, -26.4538), (-22.81, -27.3339), (-35.0, -30.0), (-45.42, -27.8771), (-57.47, -26.6437), (-70.34, -26.4538), (-82.81, -27.3339)]
    ltraj = len(traj)

    fileSD = open(nameSD,'ab')
    #fileUSB = open(nameUSB,'ab')

    measured_curves = [] 
    i = 0
    while True:
        try:
            print(f'//////--Point {i} out of {ltraj}--//////')
            curve_repetition_n = point_and_measure(traj[i],radar_measure_wait_time)

            measured_curves.append(curve_repetition_n) 
            az_real_position = az_motor.read_encoder()- az_motor.offset_angle_encoder
            el_real_position = el_motor.read_encoder() - el_motor.offset_angle_encoder 
            real_trajectory.append((az_real_position, el_real_position))
            if i == ltraj-1:
                break
            
            i += 1
        except:
            break
    pkl.dump(real_trajectory, fileSD)
    pkl.dump(measured_curves, fileSD)
    fileSD.close()
    # pkl.dump(measured_curves, fileUSB)
    # fileUSB.close()

    # Done, returning motors to 0 position
    #-------------------------------------------------------------
    az_motor.move(0)
    el_motor.move(0)
    #-------------------------------------------------------------

    # Printing results
    print("finished measurement")