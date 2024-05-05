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

def point_and_measure(angles, measure_wait_time):
    az_motor.move(angles[0])
    el_motor.move(angles[1])
    
    # Wait for motors to reach target angle
    az_motor.wait_for_target_angle(angles[0])
    el_motor.wait_for_target_angle(angles[1])
    measure_time = datetime.utcnow()

    # Waits for the radar curve to refresh
    print("Arrived to position, waiting " + str(measure_wait_time) + "s for radar curve to refresh")
    time.sleep(measure_wait_time)

    curve = serial_radar.get_curve()

    print("Measured distance at AZ = " + str(angles[0])
        +" and EL = " + str(angles[1]))
    
    return (measure_time, curve)    #distance, curve)


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

    # try:
    #     number_of_trajectories = int(args.number_of_trajectories)
    # except TypeError:
    #     number_of_trajectories = 1
    

    # Defining wait time for radar
    radar_measure_wait_time = args.radar_wait_time

    traj = hex_trajectory.hex_trajectory(RADIO_BEAM, RADIO_SURFACE)
    # for point in og_traj:
    #     angle_rep_index = 0
    #     while True:
    #         traj.append(point)
    #         if angle_rep_index == angle_rep:
    #             break
    #         angle_rep_index += 1


    time.sleep(1)
    todays_date = date.today()      

    name = 'measurements/'
    if args.filename[-4:] == '.pkl':
        name = name + args.filename[:-4]+ '_'+ str(todays_date) + '.pkl'
    else:
        name = name + args.filename+ '_' + str(todays_date) + '.pkl'
    
    try :
        # if file exist, just append data to it
        file = open(name,'rb')
        file.close()
    except FileNotFoundError:
        #if file does not exist initialize it
        file = open(name,'wb')
        config_dict = {'params': (RADIO_BEAM, RADIO_SURFACE),
                       'traj' : traj}
        pkl.dump(config_dict,file)
        file.close()

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

    az_motor = stepper_motor.stepper_motor(id = 1,speed=0, max_speed=400, acceleration=200)
    az_motor.add_limit_switch(12,-90) #12
    az_motor.add_limit_switch(25,90) #24
    time.sleep(1)

    el_motor = stepper_motor.stepper_motor(id = 2, speed=0, max_speed=400, acceleration=200)
    el_motor.add_limit_switch(18,-44) #18
    el_motor.add_limit_switch(23,44) #23
    time.sleep(1)

    # Initializing motors position using limit switches

    az_motor.initialization(dir=1,speed=0, max_speed=400, acceleration=200)
    while not az_motor.is_initialized:
        time.sleep(MOTOR_STATUS_POLLING_TIME)
    az_motor.move(0)

    el_motor.initialization(dir=1,speed=0, max_speed=400, acceleration=200) 
    while not el_motor.is_initialized:
        time.sleep(MOTOR_STATUS_POLLING_TIME)
    el_motor.move(0)
    
    print("Both motors are initialized")
    
    data_counter = 0
    
    # Pointing and measuring the corresponding angles
    # while True:
    
    ltraj = len(traj)
    file = open(name,'ab')
    measured_curves = [] 
    i = 0
    while True:
        try:
            #same_point_counter = 0
            #same_point_curves = []
            print(f'//////--Point {i} out of {ltraj}--//////')
            #while True:
            curve_repetition_n = point_and_measure(traj[i],radar_measure_wait_time)
            #same_point_curves.append(curve_repetition_n)
            #print(f'///--Repetition {same_point_counter} out of {angle_rep}--///')

            #if same_point_counter == angle_rep:
            #       break
            #   same_point_counter += 1
            #print(f'number of meas per point = {len(same_point_curves)}')
            measured_curves.append(curve_repetition_n) 
            if i == ltraj-1:
                break
            
            i += 1
        except:
            break

    pkl.dump(measured_curves,file)
    file.close()

    # Done, returning motors to 0 position
    #-------------------------------------------------------------
    az_motor.move(0)
    el_motor.move(0)
    #-------------------------------------------------------------

    # Printing results
    print("Measures:")
    print(measured_curves)