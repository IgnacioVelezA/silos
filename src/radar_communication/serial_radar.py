#!/usr/bin/env python3
import time
import serial
import numpy as np
import struct
import crcmod

# Radar parameters
MIN_DISTANCE = 0
MAX_DISTANCE = 30
CURVE_POINTS = 128
WAIT_TIME_CMDS = 0.5
WAIT_TIME_CURVE = 0.5

# Expected response length
CURVE_RESPONSE_LEN = 133
START_READING_CURVE_RESPONSE_LEN = 8
STOP_READING_CURVE_RESPONSE_LEN = 8
COMMUNICATION_TEST_RESPONSE_LEN = 7

# Initializing serial comm variable
ser = None

def open_serial(serial_port):
    """
    Configures the serial connections (the parameters differs on the device you are connecting to)
    """
    global ser
    ser = serial.Serial(
        port=serial_port,
        baudrate=9600,bytesize=8, timeout=2, parity='N', stopbits=1)

def is_serial_open():
    """
    Returns whether the serial port is open or not
    """
    return ser.isOpen()

def close_serial():
    """
    Closes the serial port
    """
    ser.close()

def send_cmd(cmd, expected_response_len, wait_time):
    """
    Send command via serial
    """
    #print("Sent: "+ str(cmd)+", Length: " + str(len(cmd)))
    recv_ok = False
    response = bytearray()
    counter = 0

    while not recv_ok:
        ser.write(cmd)
        time.sleep(wait_time)
        while ser.inWaiting() > 0:
            response += bytearray(ser.read())

        #print("Received: " +str(response)+", Length: " + str(len(response)) +"\n")
        if len(response) == expected_response_len:
            recv_ok = True
        else:
            response = bytearray()
            counter += 1
            print(f'/{counter}/Did not receive enough bytes, retrying...')

    return response

def communication_test():
    """
    Radar communication test command
    """
    cmd = bytearray([0x01, 0x66, 0xAA, 0x55, 0x00, 0x01, 0xF9, 0xCA])
    send_cmd(cmd, COMMUNICATION_TEST_RESPONSE_LEN, WAIT_TIME_CMDS)

def start_reading_curve():
    """
    Start reading curve - IMPORTANT BEFORE READING
    """
    cmd = bytearray([0x01, 0x10, 0x20, 0x34, 0x00, 0x01, 0x02, 0x00,0x01,0x42,0x26])
    send_cmd(cmd, START_READING_CURVE_RESPONSE_LEN, WAIT_TIME_CMDS)

def read_curve():
    """
    Read curve from radar
    """
    cmd = bytearray([0x01, 0x04, 0x80, 0x00, 0x00, 0x40, 0xD8, 0x3A])
    raw_curve = send_cmd(cmd, CURVE_RESPONSE_LEN, WAIT_TIME_CURVE)
    curve_array = raw_curve[3:131]
    return curve_array

def stop_reading_curve():
    """
    Stop reading curve - IMPORTANT After READING
    """
    cmd = bytearray([0x01, 0x10, 0x20, 0x34, 0x00, 0x01, 0x02, 0x00,0x00,0x83,0xE6])
    send_cmd(cmd,STOP_READING_CURVE_RESPONSE_LEN, WAIT_TIME_CMDS)

def get_curve():
    """
    Gets curve from radar.
    Also sends the command to start and stop reading the curve.
    """
    print("Starting to read curve")
    start_reading_curve()
    print("Read curve")
    curve = read_curve()
    print("Stop reading curve")
    stop_reading_curve()
   
    return curve

def high_level_adjustment(high_level_m):
    """
    Adjusts high level
    """
    level_in_bytes = bytearray(struct.pack("f", high_level_m))

    crc16 = crcmod.mkCrcFun(0x18005, rev=True, initCrc=0xFFFF, xorOut=0x0000)

    cmd_without_crc = bytearray([0x01, 0x10, 0x20, 0x4a, 0x00, 0x02, 0x04, level_in_bytes[0],level_in_bytes[1],level_in_bytes[2], level_in_bytes[3]])

    crc_msg_bytes=int.to_bytes(crc16(cmd_without_crc),2,'big')

    cmd = bytearray([0x01, 0x10, 0x20, 0x4a, 0x00, 0x02, 0x04, level_in_bytes[0],level_in_bytes[1], level_in_bytes[2], level_in_bytes[3] , crc_msg_bytes[1], crc_msg_bytes[0]])
    resp = send_cmd(cmd, START_READING_CURVE_RESPONSE_LEN, WAIT_TIME_CMDS)
    return resp

def high_level_debug(byte_array):
    """
    Adjusts high level
    """

    crc16 = crcmod.mkCrcFun(0x18005, rev=True, initCrc=0xFFFF, xorOut=0x0000)

    cmd_without_crc = bytearray([0x01, 0x10, 0x20, 0x4a, 0x00, 0x02, 0x04, byte_array[0],byte_array[1],byte_array[2], byte_array[3]])

    crc_msg_bytes=int.to_bytes(crc16(cmd_without_crc),2,'big')

    cmd = bytearray([0x01, 0x10, 0x20, 0x4a, 0x00, 0x02, 0x04, byte_array[0],byte_array[1], byte_array[2], byte_array[3] , crc_msg_bytes[1], crc_msg_bytes[0]])
    resp = send_cmd(cmd, START_READING_CURVE_RESPONSE_LEN, WAIT_TIME_CMDS)
    return resp


def low_level_adjustment(low_level_m):
    """
    Adjusts high level
    """
    level_in_bytes = bytearray(struct.pack("f", low_level_m))

    crc16 = crcmod.mkCrcFun(0x18005, rev=True, initCrc=0xFFFF, xorOut=0x0000)

    cmd_without_crc = bytearray([0x01, 0x10, 0x20, 0x48, 0x00, 0x02, 0x04, level_in_bytes[1], level_in_bytes[0]])

    crc_msg_bytes=int.to_bytes(crc16(cmd_without_crc),2,'big')

    cmd = bytearray([0x01, 0x10, 0x20, 0x48, 0x00, 0x02, 0x04, level_in_bytes[1], level_in_bytes[0] , crc_msg_bytes[1], crc_msg_bytes[0]])
    resp = send_cmd(cmd, START_READING_CURVE_RESPONSE_LEN, WAIT_TIME_CMDS)
    return resp

# for CRC calculations go to: https://www.lammertbies.nl/comm/info/crc-calculation
# crc16 = crcmod.mkCrcFun(0x18005, rev=True, initCrc=0xFFFF, xorOut=0x0000)
# print(hex(crc16(bytes([0x01, 0x04, 0x80, 0x00, 0x00, 0x40]))))
# 0x3ad8


