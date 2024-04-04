"""
This module has the raw functions to communicate with the RMDx6 motor via can
*note the can interface must be set up
"""
import can

def send_can_msg(data_to_send, device_id, can_x):
    """
    Sends a can msg to device_id with data_to_send, over can_x network.
    """
    msg_to_send = can.Message(arbitration_id= device_id, data=data_to_send, is_extended_id=False)

    can_x.send(msg_to_send)

    msg_recieved = can_x.recv(10.0)
    if msg_recieved is None:
        print('Timeout occurred, no response.')
    
    return msg_recieved

def encoder4bytes_to_value(input_array):
    """
    Transfroms 4 bytes from the encoder to value
    """
    value = input_array[0]+(input_array[1]<<8)+(input_array[2]<<16) \
    +((input_array[3]&0x7f)<<24)-(input_array[3]&0x80)/(0x80)*2**31

    return value

def value_to_4bytes(input_value):
    """
    Transfroms a value to 4 bytes, inverting byte order

    Parameters
    ----------
    input_value : int
        value to be splitted into 4 bytes

    Returns
    -------
    list
        list with the 4 bytes correctly splitted
    """
    data_0 = input_value & 0xFF
    data_1 = (input_value >> 8) & 0xFF
    data_2 = (input_value >> 16) & 0xFF
    data_3 = (input_value >> 24) & 0xFF

    return [data_0, data_1, data_2, data_3]

def value_to_2bytes(input_value):
    """
    Transfroms a value to 2 bytes, inverting byte order
    """
    data_0 = input_value & 0xFF
    data_1 = (input_value >> 8) & 0xFF

    return [data_0, data_1]

if __name__ == '__main__':
    can0 = can.interface.Bus(channel = 'can0', bustype = 'socketcan')# socketcan_native

    SPEED = 1000

    ANGLE = 0

    speed_byte_array = value_to_2bytes(SPEED)
    angle_byte_array = value_to_4bytes(ANGLE)

    #CMD = 0xA4: Mover motor, data_2 y data_3 son rapidez.
    #	data_4, _5, _6 y _7 son angulo

    #CMD = 0x92 # motor multi-turn angle
    CMD = 0x61 # raw encoder value


    #CMD = 0xA4 # move motor

    data_to_send_array = [CMD, 0x00, speed_byte_array[0], speed_byte_array[1],
        angle_byte_array[0], angle_byte_array[1], angle_byte_array[2], angle_byte_array[3]]

    response = send_can_msg(data_to_send_array,0x141,can0)
    print(response)

    encoder_val=encoder4bytes_to_value(response.data[4:8])
    print(encoder_val)
