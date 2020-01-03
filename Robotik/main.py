# Libraries
import os

import RPi.GPIO as GPIO
import time
import pickle
import numpy as np
import pandas as pd
from utils import move2pos

# GPIO Mode (Board/BCM)
GPIO.setmode(GPIO.BCM)

# Define ports ***************************
# Stepper Motors
step_door = [0, 0, 0, 0]

# DC motors
## Chasis wheels [left_in, left_out, right_in, right_out]
dc_wheel = [0, 0, 0, 0]
## Cartesian X
dc_x = [0, 0]
## Cartesian Y
dc_y = [0, 0]
## Hoisting cable
dc_z = [0,0]
## Roller
dc_roll = [0, 0]

# IR sensors
ir_left = 0
ir_right = 0

# Ultrasonic sensors
## Balloon detection
us_balloon = [0, 0]
## Main chasis
us_chasis = [0, 0]
## Cartesian System
us_x = [1, 2]
us_y = [0, 0]
## Scissor lift
us_z = [0, 0]

# Contact switch
c_switch = [,]

# Weight scale
scale = []

# Other constants
check_b = [] # lower and upper distance between the claw and balloon

# Setup GPIO *******************************
GPIO.setup(us_balloon[0], GPIO.OUT)
GPIO.setup(us_balloon[1], GPIO.IN)
GPIO.setup(us_x[0], GPIO.OUT)
GPIO.setup(us_x[1], GPIO.IN)
GPIO.setup(us_y[0], GPIO.OUT)
GPIO.setup(us_y[1], GPIO.IN)
GPIO.setup(us_z[0], GPIO.OUT)
GPIO.setup(us_z[1], GPIO.IN)
GPIO.setup(us_chasis[0], GPIO.OUT)
GPIO.setup(us_chasis[1], GPIO.IN)
# ******************************************

if __name__ == '__main__':
    # Initialization
    print("Initializing...")
    s_pins = [us_x, us_y, us_z, us_chasis, c_switch]
    m_pins = [dc_x, dc_y, dc_z, dc_wheels, step_door, dc_roll]
    # Load table
    table = pd.read_csv("positions_table.csv", index_col = 0)
    # Get initial position
    move2pos(table.iloc[0,:], , s_pins, m_pins)
    print("Initialization completed.")

    # Move to balloon postion
    # Load last checkpoint
    with open("checkpoint", 'rb') as f:
    checkpoint = pickle.load(f)

    for i in range(checkpoint, 9):
        # Go to balloon position
        move2pos(table.iloc[i,:], s_pins, m_pins) # Travel to designated position

        # Capture balloon
        move2pos(table.iloc[i+1,:], s_pins, m_pins) # Lowering
        # Case 1: A balloon is detected
        if distance(us_balloon) > check_b[0] and distance(us_balloon) < check_b[1]:
            move2pos(table.iloc[i+2,:], s_pins, m_pins) # Grab
            move2pos(table.iloc[i+3,:], s_pins, m_pins) # Lift
            
            # !!! weight measurement here !!!

            move2pos(table.iloc[i+4,:], s_pins, m_pins) # Move to end position
            move2pos(table.iloc[i+5,:], s_pins, m_pins) # Lowering
            move2pos(table.iloc[i+6,:], s_pins, m_pins) # Release
            move2pos(table.iloc[i+7,:], s_pins, m_pins) # Lift
        # Case 2: No balloon is detected
        else
            move2pos(table.iloc[i+3,:], s_pins, m_pins) # Lift

        # Save last checkpoint
        with open("checkpoint", 'wb') as f:
        pickle.dump(i, f)