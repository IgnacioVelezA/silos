import src.can_communication.motor_can as motor_can
import can
import RPi.GPIO as GPIO
import time
from datetime import datetime, timedelta

ANGLE_ERROR = 10    # 0.1 degrees

MOTOR_STATUS_POLLING_TIME = 0.1     # in seconds
MOVE_TIMEOUT = 20 #in seconds

commands = {"MOVE":0xa4, # 2.20 Absolute position closed-loop control command
            "READPOS":0x60, # 2.6 Read multi-encoder position data command
            "READORIGINALPOS":0x61, # 2.6 Read multi-encoder original position data command
            "READZEROOFFSET":0x62, # 2.6 Read multi-encoder zero offset data command
            "READANGLE": 0x92, # 2.11 Read multi-turn angle command
            "STOPMOTOR": 0x81, # 2.16 Motor stop command
            "SPEEDLOOP": 0xA2, # 2.18 Speed Closed-loop Control Command
            "BRAKERELEASE": 0x77, # 2.27 System brake release command 
            "BRAKELOCK": 0x78, # 2.27 System brake lock command
            "CANIDSETTING": 0x79, # CANID setting command
            "READPIDPARAMETERS": 0x30, # 2.1 Read PID parameter command
            "WRITEPIDRAMPARAMETERS": 0x31, # 2.2 Write PID parameters to RAM command 
}

class motor:
    def __init__(self, can_bus, id, move_speed=1):
        self.can_bus = can_bus
        self.id = id
        self.move_speed = move_speed
        self.speed_loop_speed = 0
        self.is_initialized = False
        self.limit_switch_list = []
        self.limit_switch_angle_list = []
        self.angle_offset = 0
        self.ignore_stop = False

    def initialization(self, loop_speed=500):
        """
        This routine sets the offset. Starts moving the motor slowly
        to one side until LS is reached, then it stops the motor and
        sets the corresponding LS angle offset.
        """
        # Making the motor turn
        self.speed_loop_speed = loop_speed
        self.speed_loop(self.speed_loop_speed)
        
        LS_reached = False
        # Waits until the button is pressed
        while not LS_reached:
            for LS_pin in self.limit_switch_list:
                if GPIO.event_detected(LS_pin):
                    time.sleep(0.01)
                    #Checks if the event is a LS pressed
                    if not GPIO.input(LS_pin):
                        print("A button was pressed")
                        #Checks if the limit switch is OK for initialization
                        if self.check_correct_limit_switch_stop(LS_pin):
                            LS_reached = True
                            LS_index = self.limit_switch_list.index(LS_pin)
                            LS_angle = self.limit_switch_angle_list[LS_index]
                            print("Setting offset at "+ str(LS_angle))
                        else:
                            self.ignore_stop = True
                            self.return_from_wrong_LS()
                            self.ignore_stop = False

        time.sleep(0.5)
        # Assuring the motor is correctly reading the angle
        self.read_angle(use_offset=False)
        self.read_angle(use_offset=False)
        self.read_angle(use_offset=False)
        self.read_angle(use_offset=False)
        self.read_angle(use_offset=False)
        angle_read = self.read_angle(use_offset=False)


        # Calculates and sets the offset, and sets is_initialized = True
        # LS_angle = self.LS_angle #Read at limit_switch.yaml
        self.angle_offset = int(LS_angle-angle_read)
        print("Offset used: "+ str(angle_read*(-1)))
        self.is_initialized = True
        print("Finished initializing device " + str(self.id))

    def move(self, angle, new_speed = None, use_offset = True, override_initialization = False):
        if self.is_initialized or override_initialization:
            #CHECK IF THE ANGLE IS BEYOND LIMITS
            if (angle <= max(self.limit_switch_angle_list) and angle >= min(self.limit_switch_angle_list)) or override_initialization:
                if new_speed == None:
                    new_speed = self.move_speed

                if use_offset:
                    angle_as_bytes = motor_can.value_to_4bytes(angle-self.angle_offset)
                else:
                    angle_as_bytes = motor_can.value_to_4bytes(angle)

                print("Moving to angle "+ str(angle)+", device " + str(self.id))

                speed_as_bytes = motor_can.value_to_2bytes(new_speed)
                msg = [commands["MOVE"], 0, speed_as_bytes[0], speed_as_bytes[1], \
                    angle_as_bytes[0], angle_as_bytes[1], angle_as_bytes[2], angle_as_bytes[3]]
                motor_can.send_can_msg(data_to_send = msg, device_id = self.id, can_x = self.can_bus)
            else:
                print("Target angle is beyond limit switches boundaries")
        else:
            print ("Can't do. You have to initialize the motor first")

    def speed_loop(self, new_speed = None):
        
        if new_speed == None:
            new_speed = self.speed_loop_speed
        else:
            self.speed_loop_speed=new_speed

        print("Speed looping device " +str(self.id) + " with speed " + str(new_speed))
        speed_as_bytes = motor_can.value_to_4bytes(new_speed)    
        msg = [commands["SPEEDLOOP"], 0, 0, 0, \
            speed_as_bytes[0], speed_as_bytes[1], speed_as_bytes[2], speed_as_bytes[3]]
        motor_can.send_can_msg(data_to_send = msg, device_id = self.id, can_x = self.can_bus)

    def stop(self,channel = 0):
        """
        Stops the motor. If this function is called by an interrupt, it checks if the limit switch
        is being pressed. If this function was called internally, it stops the motor.
        If the motor is initializing, started in an invalid zone and hit a wrong limit switch, 
        ignore_stop is True and the motor doesn't stop.
        """
        if channel != 0:
            if not self.ignore_stop:
                time.sleep(0.02)
                if not GPIO.input(channel):
                    print("Stopping motor, device " + str(self.id))
                    print("Stopped from channel " + str(channel))
                    msg = [commands["STOPMOTOR"], 0, 0, 0, 0, 0, 0, 0]
                    motor_can.send_can_msg(data_to_send = msg, device_id = self.id, can_x = self.can_bus)
            else:
                print("Limit switch ignored")
        else:
            print("Stopping motor")
            msg = [commands["STOPMOTOR"], 0, 0, 0, 0, 0, 0, 0]
            motor_can.send_can_msg(data_to_send = msg, device_id = self.id, can_x = self.can_bus)

    def check_correct_limit_switch_stop(self, channel):
        """
        Returns False if the motor activated the maximum angle LS while moving backwards,
        and also if it activated the minimum angle LS while moving forward.
        Returns True if neither of these things happened.
        """
        pin_index = self.limit_switch_list.index(channel)
        # If going backward and touched the max angle limit switch
        if self.speed_loop_speed < 0 and (self.limit_switch_angle_list[pin_index]
            != min(self.limit_switch_angle_list)):
            return False

        # If going forward and touched the min angle limit switch
        elif self.speed_loop_speed > 0 and (self.limit_switch_angle_list[pin_index]
            != max(self.limit_switch_angle_list)):
            return False
        else:
            return True

    def wait_for_target_angle(self, target_angle, use_offset = True):
        """
        Stays in this function until the target angle is reached.
        May move again if it takes too long to reach (because of a LS misfunction)
        """
        call_time = datetime.now()
        using_offset = use_offset
        at_final_position = False
        while not at_final_position:
            # If it takes too long to reach destination, move again
            # Added in case of limit switch misfunction... may remove later
            if datetime.now()-call_time >= timedelta(seconds=MOVE_TIMEOUT):
                call_time = datetime.now()
                self.move(target_angle, use_offset=using_offset)

            # Checks if the motor arrived to destination
            time.sleep(MOTOR_STATUS_POLLING_TIME)
            if (abs(self.read_angle(use_offset)-target_angle) < ANGLE_ERROR):
                at_final_position = True
            
        return at_final_position

    def return_from_wrong_LS(self):
        print("Returning from wrong LS")
        if self.speed_loop_speed > 0:
            # Go back 270 degrees
            time.sleep(0.01)
            actual_angle = self.read_angle(use_offset=False)
            target_angle = int(actual_angle-300*100)
            self.move(target_angle, use_offset=False, override_initialization=True)
            self.wait_for_target_angle(target_angle, use_offset=False)
            self.speed_loop()
        else:
            # Go back 270 degrees
            time.sleep(0.01)
            actual_angle = self.read_angle(use_offset=False)
            target_angle = int(actual_angle+300*100)
            self.move(target_angle,use_offset=False, override_initialization=True)
            self.wait_for_target_angle(target_angle, use_offset=False)
            self.speed_loop()

    
    def add_limit_switch(self, raspi_pin, angle):
        """
        Adds a limit switch to the motor, attached to the corresponding raspi_pin,
        with its physical angle. If the limit switch is pressed, the motor stops.
        """
        self.limit_switch_list.append(raspi_pin)
        self.limit_switch_angle_list.append(angle)
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(raspi_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.add_event_detect(raspi_pin, GPIO.BOTH, callback=self.stop, bouncetime=100)

    def read_pos(self):
        msg = [commands["READPOS"],0,0,0,0,0,0,0]
        pos_in_bytes_msg = motor_can.send_can_msg(data_to_send=msg, device_id=self.id, can_x=self.can_bus)
        pos = motor_can.encoder4bytes_to_value(pos_in_bytes_msg.data[4:8])
        return pos

    def read_original_pos(self):
        msg = [commands["READORIGINALPOS"],0,0,0,0,0,0,0]
        original_pos_in_bytes_msg = motor_can.send_can_msg(data_to_send=msg, device_id=self.id, can_x=self.can_bus)
        original_pos = motor_can.encoder4bytes_to_value(original_pos_in_bytes_msg.data[4:8])
        return original_pos

    def read_zero_offset(self):
        msg = [commands["READZEROOFFSET"],0,0,0,0,0,0,0]
        offset_in_bytes_msg = motor_can.send_can_msg(data_to_send=msg, device_id=self.id, can_x=self.can_bus)
        offset = motor_can.encoder4bytes_to_value(offset_in_bytes_msg.data[4:8])
        return offset

    def read_angle(self, use_offset = True):
        msg = [commands["READANGLE"],0,0,0,0,0,0,0]
        angle_in_bytes_msg = motor_can.send_can_msg(data_to_send=msg, device_id=self.id, can_x=self.can_bus)
        angle = motor_can.encoder4bytes_to_value(angle_in_bytes_msg.data[4:8])
        if use_offset == True:
            final_angle = angle+self.angle_offset
        else:
            final_angle = angle
        return final_angle

    def set_can_id(self, id):
        msg = [commands["CANIDSETTING"], 0, 0, 0, 0, 0, 0, id]
        
        motor_can.send_can_msg(data_to_send = msg, device_id = self.id, can_x = self.can_bus)
        self.id = 0x140+id       

    def read_can_id(self):
        msg = [commands["CANIDSETTING"], 0, 1, 0, 0, 0, 0, 0]
        motor_can.send_can_msg(data_to_send = msg, device_id = self.id, can_x = self.can_bus)        

    def read_pid_parameters(self):
        msg = [commands["READPIDPARAMETERS"], 0, 0, 0, 0, 0, 0, 0]
        pid_parameters_resp = motor_can.send_can_msg(data_to_send = msg, device_id = self.id, can_x = self.can_bus)
        current_loop_kp = pid_parameters_resp.data[2]
        current_loop_ki = pid_parameters_resp.data[3]
        speed_loop_kp = pid_parameters_resp.data[4]
        speed_loop_ki = pid_parameters_resp.data[5]
        position_loop_kp = pid_parameters_resp.data[6]
        position_loop_ki = pid_parameters_resp.data[7]
        print("Current loop KP: "+ str(current_loop_kp))
        print("Current loop KI: "+ str(current_loop_ki))
        print("Speed loop KP: "+ str(speed_loop_kp))
        print("Speed loop KI: "+ str(speed_loop_ki))
        print("Position loop KP: "+ str(position_loop_kp))
        print("Position loop KI: "+ str(position_loop_ki))      
         
    def write_pid_ram_parameters(self, current_loop_kp, current_loop_ki, 
        speed_loop_kp, speed_loop_ki, position_loop_kp, position_loop_ki):
        print("Writing PID parameters to RAM, device " + str(self.id))
        msg = [commands["WRITEPIDRAMPARAMETERS"], 0, current_loop_kp, current_loop_ki,
            speed_loop_kp, speed_loop_ki, position_loop_kp, position_loop_ki]
        motor_can.send_can_msg(data_to_send = msg, device_id = self.id, can_x = self.can_bus)

    def reset(self):
        print("Resetting device " + str(self.id))
        self.limit_switch_list = []
        self.limit_switch_angle_list = []
        self.angle_offset = 0
        self.is_initialized = False

if __name__ == '__main__':
    # Initializing radar motors, setting PID and limit switches
    GPIO.cleanup()
    can_0 = can.interface.Bus(channel = 'can0', bustype = 'socketcan')# socketcan_native
    az_motor = motor(can_bus = can_0, id = 0x140, move_speed = 1)
    az_motor.add_limit_switch(18,-95*100)
    az_motor.add_limit_switch(23,95*100)
    time.sleep(1)
    az_motor.write_pid_ram_parameters(100, 100, 255, 100, 5, 5)

    el_motor = motor(can_bus = can_0, id = 0x145, move_speed = 1)
    el_motor.add_limit_switch(24,43.8*100)
    el_motor.add_limit_switch(12,-36.6*100)
    time.sleep(1)
    el_motor.write_pid_ram_parameters(100, 100, 240, 100, 5, 5)
