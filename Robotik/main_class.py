"""
this class contain the constructor class to run the 3 setting programm
test 1: testing the balloon detection and fetching
test 2: testing the balloon classification and dropping
main task: the complete program of the robot
"""

import pandas as pd
import os
import RPi.GPIO as GPIO
import time

class MainClass:
    """
     defining the Class constant Properties containing the pins
    """
    # pins of the x axis motor [Forward, Backward]
    PIN_M_X_AXIS = [1, 2]
    # pins of the y axis motor [Forward, Backward]
    PIN_M_Y_AXIS = [1, 2]
    # pins of the z axis motor [Forward, Backward]
    PIN_M_Z_AXIS = [1, 2]
    # pins of the x axis ultrasonic
    PIN_S_X_AXIS = {'trigger': 1, 'echo': 2}
    # pins of the y axis ultrasonic
    PIN_S_Y_AXIS = {'trigger': 1, 'echo': 2}
    # pins of the z axis ultrasonic
    PIN_S_Z_AXIS = {'trigger': 1, 'echo': 2}
    # pins of the car motor [Forward, Backward]
    PIN_M_CAR = [1, 2]
    # pin of the IR sensor in the back
    PIN_IR_BACK = [1]
    # pin of the IR sensor in the front
    PIN_IR_FRONT= [1]
    # pins of the stepper motor
    PIN_STEPPER = [1, 2, 3, 4]
    STEPPER_STEPS = [[1, 0, 0, 0],
                     [1, 1, 0, 0],
                     [0, 1, 0, 0],
                     [0, 1, 1, 0],
                     [0, 0, 1, 0],
                     [0, 0, 1, 1],
                     [0, 0, 0, 1],
                     [1, 0, 0, 1]]
    # pin of the roll dc motor [Forward, Backward]
    PIN_M_ROLL = [1, 2]


    def __init__(self):
        """
        constructor of the main class
        """
        ### set class properties #######
        self.test1_button = False
        self.test2_button = False
        self.main_task_button = False
        self.ball_start_pos = pd.DataFrame
        self.end_pos_s = pd.DataFrame
        self.end_pos_m = pd.DataFrame
        self.end_pos_l = pd.DataFrame
        self.s_ball_counter = 0
        self.m_ball_counter = 0
        self.l_ball_counter = 0
        self.ball_gotten = 0

        self.setup_pins()
        self.run()

    def setup_pins(self):
        """
        this method will setup the pis of the raspberry pie
        """
        GPIO.cleanup()
        # set motor pins as output
        GPIO.setup(MainClass.PIN_M_X_AXIS, GPIO.OUT)
        GPIO.setup(MainClass.PIN_M_Y_AXIS, GPIO.OUT)
        GPIO.setup(MainClass.PIN_M_Z_AXIS, GPIO.OUT)
        GPIO.setup(MainClass.PIN_M_CAR, GPIO.OUT)
        GPIO.setup(MainClass.PIN_M_ROLL, GPIO.OUT)
        GPIO.setup(MainClass.PIN_STEPPER, GPIO.OUT)

        # set ultrasonic pins
        GPIO.setup(MainClass.PIN_S_X_AXIS['trigger'], GPIO.OUT)
        GPIO.setup(MainClass.PIN_S_X_AXIS['echo'], GPIO.IN)
        GPIO.setup(MainClass.PIN_S_Y_AXIS['trigger'], GPIO.OUT)
        GPIO.setup(MainClass.PIN_S_Y_AXIS['echo'], GPIO.IN)
        GPIO.setup(MainClass.PIN_S_Z_AXIS['trigger'], GPIO.OUT)
        GPIO.setup(MainClass.PIN_S_Z_AXIS['echo'], GPIO.IN)

        # set IR sensor pins
        GPIO.setup(MainClass.PIN_IR_FRONT, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(MainClass.PIN_IR_BACK, GPIO.IN, pull_up_down=GPIO.PUD_UP)


    def read_csv_position_file(self):
        """
        this method will import the postion from the csv files
        """
        csvpath_strat = os.path.join(os.getcwd(), 'position.csv')
        csvpath_end = os.path.join(os.getcwd(), 'end_position.csv')

        self.ball_start_pos = pd.read_csv(csvpath_strat, sep=';', header=[0])
        ball_end_pos = pd.read_csv(csvpath_end, sep=';', header=[0])
        self.end_pos_s = ball_end_pos.loc[ball_end_pos['size'] == 's'].reset_index(drop=True)
        self.end_pos_m = ball_end_pos.loc[ball_end_pos['size'] == 'm'].reset_index(drop=True)
        self.end_pos_l = ball_end_pos.loc[ball_end_pos['size'] == 'l'].reset_index(drop=True)


    def check_run_mode(self):
        """
        this method will check for the needed running mode
        """

        # check if the test one button is choosen
        if GPIO.pin(test1_button == 1):
            # check if the test one button is choosen
            self.test1_button = True
            self.test2_button = False
            self.main_task_button = False
        elif GPIO.pin(test2_button == 1):
            # check if the test two button is choosen
            self.test1_button = False
            self.test2_button = True
            self.main_task_button = False
        elif GPIO.pin(main_task_button == 1):
            # check if the main task button is choosen
            self.test1_button = False
            self.test2_button = False
            self.main_task_button = True

    def run(self):
        """
        this method will check the choosen mode and run it
        """
        while True:
            # uptade positions
            self.read_csv_position_file()
            # check for a choosen mode
            self.check_run_mode()

            if self.test1_button:
                self.test1_mode()

            elif self.test2_button:
                self.test2_mode()

            elif self.main_task_button:
                self.main_mode()

    def test1_mode(self):
        """
        this method will define the step of the test 1 mode
        """

    def test2_mode(self):
        """
        this method will define the step of the test 2 mode
        """

    def main_mode(self):
        """
        this method will define the step of the main mode
        """
        self.move_car(MainClass.PIN_M_CAR,MainClass.PIN_IR_BACK,'forward')
        for cnter in range(self.ball_gotten, 9):
            x_start_ball = self.ball_start_pos['X'][cnter]
            y_start_ball = self.ball_start_pos['Y'][cnter]
            z_start_ball = 1200
            self.run_dc_motor_koordinate(MainClass.PIN_M_X_AXIS, MainClass.PIN_S_X_AXIS, 'forward', x_start_ball)
            self.run_dc_motor_koordinate(MainClass.PIN_M_Y_AXIS, MainClass.PIN_S_Y_AXIS, 'forward', y_start_ball)
            self.run_dc_motor_koordinate(MainClass.PIN_M_Z_AXIS, MainClass.PIN_S_Z_AXIS, 'forward', z_start_ball)

            self.run_claw_motors(MainClass.PIN_STEPPER, MainClass.PIN_M_ROLL, 'close', 'button_pins')

            self.run_dc_motor_koordinate(MainClass.PIN_M_Z_AXIS, MainClass.PIN_S_Z_AXIS, 'backward', 50)
            self.run_dc_motor_koordinate(MainClass.PIN_M_Y_AXIS, MainClass.PIN_S_Y_AXIS, 'backward', 50)
            self.run_dc_motor_koordinate(MainClass.PIN_M_X_AXIS, MainClass.PIN_S_X_AXIS, 'backward', 50)

            weight = self.get_ballon_weight()

            if weight == 1:
                x_end = self.end_pos_s['X'][self.s_ball_counter]
                y_end = self.end_pos_s['Y'][self.s_ball_counter]
                z_end = self.end_pos_s['Z'][self.s_ball_counter]
                self.s_ball_counter = self.s_ball_counter + 1

            elif weight == 1.5:
                x_end = self.end_pos_m['X'][self.m_ball_counter]
                y_end = self.end_pos_m['Y'][self.m_ball_counter]
                z_end = self.end_pos_m['Z'][self.m_ball_counter]
                self.m_ball_counter = self.m_ball_counter + 1

            elif weight == 2:
                x_end = self.end_pos_l['X'][self.l_ball_counter]
                y_end = self.end_pos_l['Y'][self.l_ball_counter]
                z_end = self.end_pos_l['Z'][self.l_ball_counter]
                self.l_ball_counter = self.l_ball_counter + 1

            self.move_car(MainClass.PIN_M_CAR, MainClass.PIN_IR_FRONT, 'forward')

            self.run_dc_motor_koordinate(MainClass.PIN_M_X_AXIS, MainClass.PIN_S_X_AXIS, 'forward', x_end)
            self.run_dc_motor_koordinate(MainClass.PIN_M_Y_AXIS, MainClass.PIN_S_Y_AXIS, 'forward', y_end)
            self.run_dc_motor_koordinate(MainClass.PIN_M_Z_AXIS, MainClass.PIN_S_Z_AXIS, 'forward', z_end)
            self.run_claw_motors(MainClass.PIN_STEPPER, MainClass.PIN_M_ROLL, 'open', 'button_pins')
            self.run_dc_motor_koordinate(MainClass.PIN_M_Z_AXIS, MainClass.PIN_S_Z_AXIS, 'backward', 50)
            self.run_dc_motor_koordinate(MainClass.PIN_M_Y_AXIS, MainClass.PIN_S_Y_AXIS, 'backward', 50)
            self.run_dc_motor_koordinate(MainClass.PIN_M_X_AXIS, MainClass.PIN_S_X_AXIS, 'backward', 50)
            self.ball_gotten = cnter+1
            self.move_car(MainClass.PIN_M_CAR, MainClass.PIN_IR_BACK, 'backward')

        self.s_ball_counter = 0
        self.m_ball_counter = 0
        self.l_ball_counter = 0
        self.ball_gotten = 0

    def run_dc_motor_koordinate(self, motor_pins, us_pins, direction, goal_distance):
        """
        this method will run the dc motor in a defined direction and till a defined distance
        """
        move = True
        while move:
            if direction.lower() is 'forward':
                if GPIO.input(motor_pins[0]) == 0:
                    GPIO.output(motor_pins, (GPIO.HIGH, GPIO.LOW))

                dist = self.get_distance(us_pins)
                if dist >= goal_distance:
                    GPIO.output(motor_pins, (GPIO.LOW, GPIO.LOW))
                    move = False
            elif direction.lower() is 'backward':
                if GPIO.input(motor_pins[1]) == 0:
                    GPIO.output(motor_pins, (GPIO.LOW, GPIO.HIGH))

                dist = self.get_distance(us_pins)
                if dist <= goal_distance:
                    GPIO.output(motor_pins, (GPIO.LOW, GPIO.LOW))
                    move = False

    def move_car(self, motor_pins, ir_pin, direction):
        """
        this method will run the car dc motor and stop them when the IR sensor is triggered
        """
        move = True
        while move:
            if direction.lower() is 'forward':
                if GPIO.input(motor_pins[0]) == 0:
                    GPIO.output(motor_pins, (GPIO.HIGH, GPIO.LOW))
                if GPIO.input(ir_pin) == 0:
                    GPIO.output(motor_pins, (GPIO.LOW, GPIO.LOW))
                    move = False

            if direction.lower() is 'backward':
                if GPIO.input(motor_pins[1]) == 0:
                    GPIO.output(motor_pins, (GPIO.LOW, GPIO.HIGH))
                if GPIO.input(ir_pin) == 0:
                    GPIO.output(motor_pins, (GPIO.LOW, GPIO.LOW))
                    move = False

    def get_distance(self, us_pins):
        """
        ths method will trigger the ultrasonic sensor and return the distance in mm
        """
        # set Trigger to HIGH
        GPIO.output(us_pins['trigger'], GPIO.HIGH)

        # set Trigger after 0.01ms to LOW
        time.sleep(0.00001)
        GPIO.output(us_pins['trigger'], GPIO.LOW)

        start_time = time.time()
        stop_time = time.time()

        # save StartTime
        while GPIO.input(us_pins['echo']) == 0:
            start_time = time.time()

        # save time of arrival
        while GPIO.input(us_pins['trigger']) == 1:
            stop_time = time.time()

        # time difference between start and arrival
        time_elapsed = stop_time - start_time
        # multiply with the sonic speed (343000 mm/s)
        # and divide by 2, because there and back
        distance = (time_elapsed * 343000) / 2

        return distance

    def run_claw_motors(self, stepper_pins, dc_pins, movement, button_pins):
        """

        """
        if movement.lower() is 'close':
            move = True
            if GPIO.input(dc_pins[0]) == 0:
                GPIO.output(dc_pins, (GPIO.HIGH, GPIO.LOW))
            while move:
                for ind in range(8):
                    GPIO.output(stepper_pins, MainClass.STEPPER_STEPS[ind])
                    if GPIO.input(button_pins) == 1:
                        move = False
            GPIO.output(dc_pins, (GPIO.LOW, GPIO.LOW))
            GPIO.output(stepper_pins, [0, 0, 0, 0])

        if movement.lower() is 'open':
            move = True
            while move:
                for ind in range(7, -1, -1):
                    GPIO.output(stepper_pins, MainClass.STEPPER_STEPS[ind])
                    if GPIO.input(button_pins) == 1:
                        move = False
            GPIO.output(dc_pins, (GPIO.LOW, GPIO.LOW))
            GPIO.output(stepper_pins, [0, 0, 0, 0])

    def get_ballon_weight(self):
        """

        """
        weight = 1
        return weight

if __name__ == "__main__":
    MainClass()
