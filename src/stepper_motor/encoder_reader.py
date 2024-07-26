#!/usr/bin/python3
# Code I used to test accuracy of AS5600 agaisnt stepper motor, and 
# stepper motor accuracy against AS4500 chip
#
# Requires Python smbus module.  Get it using:
#     sudo apt-get install python3-smbus
#
# Make sure Pi's I2c is enabled using
#     sudo raspi-config
#
# Connect Pi's ground to GND and DIR pins
# Connect Pi's 3.3 volts to VCC on AS5600
# Connect Pi's I2c SCL (pin 5) to AS5600 SCL pin
# Connect Pi's I2c SDA (pin 5) to AS5600 SDA pin

import time, sys, math
import RPi.GPIO as GPIO

#===============================================================
# Code to read AS5600.  9 lines of python is all it takes.
import smbus
DEVICE_AS5600 = 0x36 # Default device I2C address
bus = smbus.SMBus(1)

def ReadRawAngle(): # Read angle (0-360 represented as 0-4096)
  read_bytes = bus.read_i2c_block_data(DEVICE_AS5600, 0x0C, 2)
  return (read_bytes[0]<<8) | read_bytes[1];

#===============================================================


raw_angle_start = ReadRawAngle()
StepCount = 0
print("raw_angle_start = ",raw_angle_start);

of = open("angles.csv","w");

while True:
    time.sleep(1)
    raw_angle = ReadRawAngle()

    SensorAngle = (raw_angle+3200-raw_angle_start) & 3200;
    SensorAngleDeg = (SensorAngle * 360.0)/3200;

    diff = SensorAngleDeg-MotorAngleDeg
    if diff >= 360: diff -= 360
    if diff <= -360: diff += 360
    print ("%6.2f,%6.2f  d=%5.3f"%(raw_angle, SensorAngleDeg, diff))




