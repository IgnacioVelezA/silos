# This is the main script for silos measurement. Consists of 13 different points.
from src.radar_communication import serial_radar
from src.stepper_motor import serial_arduino
from src.stepper_motor import stepper_motor
import RPi.GPIO as GPIO
import time
import math
import numpy as np
from datetime import datetime, timedelta, date
import csv
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

    # Waits for the radar curve to refresh
    print("Arrived to position, waiting " + str(measure_wait_time) + "s for radar curve to refresh")
    time.sleep(measure_wait_time)

    curve = serial_radar.get_curve()

    print("Measured distance at AZ = " + str(angles[0]) + " and EL = " + str(angles[1]))
    
    return measure_time, curve

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='moves the radar to its 13 positions and logs the info')
    
    parser.add_argument('filename', help='name of the file')
    parser.add_argument('arduino_usb_port', help='arduino usb port for serial comm, typically /dev/ttyUSB1')
    parser.add_argument('-r0','--radio_0', help='Radars HPBW')
    parser.add_argument('-rmax','--radio_max', help='Radius of circular surface to map')
    parser.add_argument('radar_usb_port', help='radar usb port for serial comm, typically /dev/ttyUSB0')
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

    traj = hex_trajectory.hex_trajectory(RADIO_BEAM, RADIO_SURFACE, set_offset_el=False)

    time.sleep(1)
    todays_date = date.today()      
    params = 'r0:'+str(RADIO_BEAM)+'_'+'rmax:'+str(RADIO_SURFACE)
    # file name
    nameSD = 'measurements/'
    if args.filename[-4:] == '.csv':
        suffix = args.filename[:-4] + '_' + params + '_' + str(todays_date) + '.csv'
    else:
        suffix = args.filename + '_' + params + '_' + str(todays_date) + '.csv'
    
    nameSD = nameSD + suffix

    # Initializing radar serial comm
    serial_radar.open_serial(args.radar_usb_port)
    if serial_radar.is_serial_open():
        print("Serial radar port open successfully")
    else:
        print("Could not open serial radar port, FATAL ERROR")

    # Initializing radar motors, setting PID and limit switches
    GPIO.cleanup()
    serial_arduino.open_serial(args.arduino_usb_port)
    if serial_arduino.is_serial_open():
        print("Serial arduino port open successfully")
    else:
        print("Could not open arduino radar port, FATAL ERROR")

    az_motor = stepper_motor.stepper_motor(id=1, speed=0, max_speed=400, acceleration=600)
    az_motor.add_limit_switch(12, -90)
    az_motor.add_limit_switch(22, 92)
    time.sleep(1)

    el_motor = stepper_motor.stepper_motor(id=2, speed=0, max_speed=400, acceleration=600)
    el_motor.add_limit_switch(18, -44)
    el_motor.add_limit_switch(23, 43)
    time.sleep(1)

    # Initializing motors position using limit switches
    az_motor.initialization(dir=1, speed=0, max_speed=400, acceleration=600)
    while not az_motor.is_initialized:
        time.sleep(MOTOR_STATUS_POLLING_TIME)
    time.sleep(0.1)
    az_motor.move(0)

    el_motor.initialization(dir=1, speed=0, max_speed=400, acceleration=600) 
    while not el_motor.is_initialized:
        time.sleep(MOTOR_STATUS_POLLING_TIME)
    time.sleep(0.1)
    el_motor.move(0)
    
    print("Both motors are initialized")

    az_LS_position = az_motor.LS_angle_meassured
    el_LS_position = el_motor.LS_angle_meassured

    # Initialize CSV file with headers
    with open(nameSD, mode='w', newline='') as fileSD:
        writer = csv.writer(fileSD)
        writer.writerow(['Measure Time', 'Commanded trajectory', 'Real trajectory', 'Curve', 'az LS', az_LS_position, 'el LS', el_LS_position])    
    
    # Pointing and measuring the corresponding angles
    ltraj = len(traj)

    measured_curves = [] 
    i = 0
    max_attempts = 5

    while True:
        try:
            print(f'//////--Point {i} out of {ltraj}--//////')
            measure_time, curve = point_and_measure(traj[i], radar_measure_wait_time)

            curve = np.array(curve)

            measured_curves.append((measure_time, traj[i], curve))
            
        except Exception as error:
            # handle the exception
            print("An exception occurred:", error)
            break

        for attempt_i in range(max_attempts):
            try:
                az_real_position = az_motor.read_encoder()
                break
            except:
                print('An error occurred while reading encoders')
                time.sleep(0.1)
                if attempt_i == max_attempts - 1:
                    print(f'Max attempts done, using nominal position for point {i} in az')
                    az_real_position = 5600

        for attempt_i in range(max_attempts):
            try:
                el_real_position = el_motor.read_encoder()
                break
            except:
                print('An error occurred while reading encoders')
                time.sleep(0.1)
                if attempt_i == max_attempts - 1:
                    print(f'Max attempts done, using nominal position for point {i} in el')
                    el_real_position =5601
                        
        real_position = (az_real_position, el_real_position)

        with open(nameSD, mode='a', newline='') as fileSD:
            writer = csv.writer(fileSD)
            writer.writerow([measure_time, traj[i], real_position, curve])
        print(real_position)
        # real_trajectory.append(real_position)

        if i == ltraj-1:
            break
        
        i += 1


    # Done, returning motors to 0 position
    az_motor.move(0)
    el_motor.move(0)

    # Printing results
    print(measured_curves)
    print("finished measurement")

    open('src/scripts/backgroundScripts/script_A_finished.flag', 'w').close()