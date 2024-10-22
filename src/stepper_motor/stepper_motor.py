import src.stepper_motor.serial_arduino as serial_arduino
import RPi.GPIO as GPIO
import time
from datetime import datetime, timedelta
import argparse
import smbus

ANGLE_ERROR = 1    # 1 degrees

MOTOR_STATUS_POLLING_TIME = 0.1     # in seconds
MOVE_TIMEOUT = 20 #in seconds
STEPS_PER_LOOP = 200 # 200 steps for 360 degrees
SERIAL_WAIT_TIME = 0.1

STANDARD_SPEED = 2
STANDARD_MAX_SPEED = 4
STANDARD_ACCELERATION = 6


ENCODER_ADRESS = 0x36 

class stepper_motor:
    def __init__(self, id, speed=STANDARD_SPEED, max_speed=STANDARD_MAX_SPEED, acceleration=STANDARD_ACCELERATION):
        self.id = id
        self.is_initialized = False
        self.limit_switch_list = []
        self.limit_switch_angle_list = []
        self.angle_offset = 0
        self.LS_angle_meassured = 0
        self.ignore_stop = False
        self.speed = speed
        self.max_speed = max_speed
        self.acceleration = acceleration


    def initialization(self, dir=1, max_speed = 2, speed=1, acceleration=10):
        """
        This routine sets the offset. Starts moving the motor slowly
        to one side until LS is reached, then it stops the motor and
        sets the corresponding LS angle offset.
        """
        #setting slow speeds
        self.set_motor_max_speed(max_speed)
        self.set_motor_speed(speed)
        self.set_motor_acceleration(acceleration)
        # Making the motor turn

        target_angle = self.read_angle()+360*dir
        self.move(target_angle,override_initialization=True)

        LS_reached = False
        # Waits until the button is pressed
        while not LS_reached:
            for LS_pin in self.limit_switch_list:
                #---------------------------------------------------
                if GPIO.event_detected(LS_pin):
                     #Checks if the event is a LS pressed
                    time.sleep(0.1)
                    if not GPIO.input(LS_pin):
                        print("A button was pressed")
                        #Checks if the limit switch is OK for initialization
                        if self.check_correct_limit_switch_stop(LS_pin,dir):
                            LS_reached = True
                            LS_index = self.limit_switch_list.index(LS_pin)
                            LS_angle = self.limit_switch_angle_list[LS_index]
                            print("Setting offset at "+ str(LS_angle))
                        else:
                            self.ignore_stop = True
                            self.return_from_wrong_LS(dir)
                            self.ignore_stop = False


        # Assuring the motor is correctly reading the angle
        angle_read = self.read_angle(use_offset=False)
        time.sleep(0.1)

        # Calculates and sets the offset, and sets is_initialized = True
        print("Finished initializing device " + str(self.id))
        self.angle_offset = LS_angle-angle_read
        self.is_initialized = True

        #setting normal speeds
        self.stop()
        time.sleep(0.1)
        self.set_motor_max_speed(self.max_speed)
        time.sleep(0.1)
        self.set_motor_speed(self.speed)
        time.sleep(0.1)
        self.set_motor_acceleration(self.acceleration)
        time.sleep(0.1)
        self.LS_angle_meassured = self.read_encoder()
        time.sleep(0.1)
        print('Encoder position for LS = '+str(self.LS_angle_meassured))


    def move(self, angle, use_offset = True, override_initialization = False):
        if self.is_initialized or override_initialization:
            #CHECK IF THE ANGLE IS BEYOND LIMITS
            if (angle <= max(self.limit_switch_angle_list) and angle >= min(self.limit_switch_angle_list)) or override_initialization:
                if use_offset:
                    target_angle = angle-self.angle_offset
                else:
                    target_angle = angle

                #Rounding angle
                target_angle = int(target_angle*100)/100

                print("Moving to angle "+ str(angle)+", device " + str(self.id))

                serial_arduino.send_cmd(self.id, "move",target_angle, SERIAL_WAIT_TIME)
            else:
                print("Target angle is beyond limit switches boundaries")
        else:
            print ("Can't do. You have to initialize the motor first")


    def stop(self,channel = 0):
        """
        Stops the motor. If this function is called by an interrupt, it checks if the limit switch
        is being pressed. If this function was called internally, it stops the motor.
        If the motor is initializing, started in an invalid zone and hit a wrong limit switch,
        ignore_stop is True and the motor doesn't stop.
        """
        if channel != 0:
            print("Limit switch ignored")
        else:
            print("Stopping motor")
            serial_arduino.send_cmd(self.id,"stop","",SERIAL_WAIT_TIME)


    def check_correct_limit_switch_stop(self, channel,dir):
        """
        Returns False if the motor activated the maximum angle LS while moving backwards,
        and also if it activated the minimum angle LS while moving forward.
        Returns True if neither of these things happened.
        """
        pin_index = self.limit_switch_list.index(channel)
        # If going backward and touched the max angle limit switch
        if dir < 0 and (self.limit_switch_angle_list[pin_index]
            != min(self.limit_switch_angle_list)):
            return False

        # If going forward and touched the min angle limit switch
        elif dir > 0 and (self.limit_switch_angle_list[pin_index]
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


    def return_from_wrong_LS(self,dir):
        print("Returning from wrong LS")
        if dir > 0:
            # Go back 280 degrees
            time.sleep(0.01)
            actual_angle = self.read_angle(use_offset=False)
            target_angle = int(actual_angle-280)
            self.move(target_angle, use_offset=False, override_initialization=True)
            self.wait_for_target_angle(target_angle, use_offset=False)
            self.move(target_angle+360)
        else:
            # Go back 270 degrees
            time.sleep(0.01)
            actual_angle = self.read_angle(use_offset=False)
            target_angle = int(actual_angle+280)
            self.move(target_angle,use_offset=False, override_initialization=True)
            self.wait_for_target_angle(target_angle, use_offset=False)
            self.move(target_angle-360)


    def add_limit_switch(self, raspi_pin, angle):
        """
        Adds a limit switch to the motor, attached to the corresponding raspi_pin,
        with its physical angle. If the limit switch is pressed, the motor stops.
        """
        self.limit_switch_list.append(raspi_pin)
        self.limit_switch_angle_list.append(angle)  # Set pull_up to True to enable the internal pull-up resistor
        #--------------------------------------------------------------
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(raspi_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.add_event_detect(raspi_pin, GPIO.BOTH, callback=self.stop, bouncetime=100)
        # -----------------------------------------------------------------


    def read_angle(self, use_offset = True):
        angle_response = serial_arduino.send_cmd(self.id,"rdangle","",SERIAL_WAIT_TIME)
        recv_index = angle_response.find("<" + str(self.id)+"-ok:")
        angle = None
        if recv_index != -1:
            end_index = angle_response.find(">")
            angle_str = angle_response[recv_index+5+len(str(self.id)):end_index]
            angle = float(angle_str)
        else:
            angle = None
            return -69420

        if use_offset == True:
            final_angle = angle+self.angle_offset
        else:
            final_angle = angle
        return final_angle


    def read_encoder(self, attempts = 5):
        bus_id = self.id -1 # motor 2 -> i2c bus 1, motor 1 -> i2c 0
        bus = smbus.SMBus(bus_id)

        for attempt_i in range(attempts):
            try:
                read_bytes = bus.read_i2c_block_data(ENCODER_ADRESS, 0x0C, 2)
                raw_angle = (read_bytes[0]<<8) | read_bytes[1];
                break
            except:
                print('An error occurred while reading encoders')
                time.sleep(0.1)
                if attempt_i == attempts - 1:
                    print(f'Max attempts done, using nominal position for point in motor {id}' )
                    raw_angle = 5600 + bus_id          

        return raw_angle


    # def get_encoder_angle(self):

    #     # Convertir bits a grados
    #     measure_degrees = self.read_encoder() #angle fmeasured by the encoders internal axis
    #     LS_degrees = self.LS_angle_meassured
    #     LS_pos_degrees = 90- self.limit_switch_angle_list[1] #for example, the LS angle is 107 but i need it to be -17 to work

    #     # Calcular la posición medida desde el LS
    #     measure_from_LS = measure_degrees - LS_degrees
        
    #     # Ajustar con la posición del LS
    #     position_corrected = measure_from_LS + LS_pos_degrees
        
    #     # Normalizar el ángulo en el rango -180 a 180
    #     angle = position_corrected % 360
    #     if angle > 180:
    #         angle -= 360
    #     elif angle < -180:
    #         angle += 360
        
    #     return angle


    def set_id(self, id):
        self.id = id


    def set_motor_speed(self,speed):
        serial_arduino.send_cmd(self.id,"setspd",speed,SERIAL_WAIT_TIME)


    def set_motor_max_speed(self,max_speed):
        serial_arduino.send_cmd(self.id,"setmaxspd",max_speed,SERIAL_WAIT_TIME)


    def set_motor_acceleration(self,acceleration):
        serial_arduino.send_cmd(self.id,"setacc",acceleration,SERIAL_WAIT_TIME)


    def reset(self):
        print("Resetting device " + str(self.id))
        self.limit_switch_list = []
        self.limit_switch_angle_list = []
        self.angle_offset = 0
        self.is_initialized = False


if __name__ == '__main__':
    # Initializing radar motors, setting PID and limit switches
    #try:
    parser = argparse.ArgumentParser(description='for debugging stepper motors')
    parser.add_argument('arduino_usb_port', help='arduino usb port for serial comm, tipically /dev/ttyUSB1')

    args = parser.parse_args()

    serial_arduino.open_serial(args.arduino_usb_port)
    if serial_arduino.is_serial_open():
        print("Serial arduino port open succesfully")
    else:
        print("Could not open arduino radar port, FATAL ERROR")
    time.sleep(1)

    GPIO.cleanup()
    az_motor = stepper_motor(id = 1,speed=0, max_speed=400, acceleration=600)
    az_motor.add_limit_switch(12,-97)
    az_motor.add_limit_switch(22,97)#¬24¬22
    time.sleep(1)

    el_motor = stepper_motor(id = 2, speed=0, max_speed=400, acceleration=600)
    el_motor.add_limit_switch(18,-32)
    el_motor.add_limit_switch(23,32)#23


    time.sleep(1)
