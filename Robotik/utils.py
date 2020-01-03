import RPi.GPIO as GPIO
from gpiozero import Robot
import time
import numpy as np

def move2position(positions, s_pins, m_pins):
    """ Move the robot and claw to the specified position.

    Arg: 
        positions: array of positions [x_axis, y_axis, z_axis, chasis]
        s_pins: sensor pins [us_x, us_y, us_z, us_chasis, c_switch]
        m_pins: motor pins [dc_x, dc_y, dc_z, dc_wheel_l, dc_wheel_r]
    Return: None
    """
    # Declare flags
    flags = isxnan(positions)

    # movement along x-axis
    while not flags[0]:
        if distance(s_pins[0]) > positions[0]:
            # move backwards
            dc_motor(m_pins[0])
        elif distance(s_pins[0]) < positions[0]:
            # move forwards
            dc_motor(m_pins[0], True)
        else:
            flags[0] = True

    # movement along y-axis
    while not flags[1]:
        if distance(s_pins[1]) > positions[1]:
            # move to the right
            dc_motor(m_pins[1])
        elif distance(s_pins[1]) < positions[1]:
            # move to the right
            dc_motor(m_pins[1], True)
        else:
            flags[1] = True

    # movement along z-axis
    while not flags[2]:
        if distance(s_pins[2]) > positions[2]:
            # move upwards
            dc_motor(m_pins[2])
        elif distance(s_pins[2]) < positions[2]:
            # move upwards
            dc_motor(m_pins[2], True)
        else:
            flags[2] = True

    # chasis movement
    while not flags[3]:
        if distance(s_pins[3]) > positions[3]:
            # !!! move backwards
            chasis = Robot(left=(m_pins[3][0],m_pins[3][1]), right=(m_pins[3][2],m_pins[3][3]))
            chasis.backward()
            sleep(0.5)
            chasis.stop()
        elif distance(s_pins[3]) > positions[3]:
            # !!! move backwards
            chasis = Robot(left=(m_pins[3][0],pins[3][1]), right=(pins[3][2],pins[3][3]))
            chasis.forward()
            sleep(0.5)
            chasis.stop()
        else:
            flags[3] = True

    # flap movement
    while not flags[4]:
        # open flap
        if positions[4] == 1:
            while(GPIO.input(s_pins[4][0]) == GPIO.LOW)
                dc_motors((m_pins[4],m_pins[5]))
                flags[4] = True
        else
            while(GPIO.input(s_pins[4][1]) == GPIO.LOW)
                dc_motors((m_pins[4],m_pins[5]), True)
                flags[4] = True

def dc_motor(pins, counter=False, step=0.5):
    """ Drive the motor CW or CCW

    Arg:
        pins: array of size 2 [in1, in2]
        counter: 0 for CW, 1 for CCW
    Return:
        None
    """
    if pins.size == 1:
        if counter:
            GPIO.output(pins[1], GPIO.HIGH)
            GPIO.output(pins[0], GPIO.LOW)
            sleep(step)
            GPIO.output(pins[1], GPIO.LOW)
            GPIO.output(pins[0], GPIO.LOW)
        else:
            GPIO.output(pins[0], GPIO.HIGH)
            GPIO.output(pins[1], GPIO.LOW)
            sleep(step)
            GPIO.output(pins[0], GPIO.LOW)
            GPIO.output(pins[1], GPIO.LOW)

def dc_motors(pins, counter=False, step=0.5):
    """ Drive the motor CW or CCW

    Arg:
        pins: array of size 2 [in1, in2]
        counter: 0 for CW, 1 for CCW
    Return:
        None
    """
    if counter:
        GPIO.output((pins[0][0],pins[1][0]), GPIO.HIGH)
        GPIO.output((pins[1][1],pins[1][1]), GPIO.LOW)
        sleep(step)
        GPIO.output((pins[0][0],pins[1][0]), GPIO.LOW)
        GPIO.output((pins[1][1],pins[1][1]), GPIO.LOW)
    else:
        GPIO.output((pins[0][0],pins[1][0]), GPIO.HIGH)
        GPIO.output((pins[1][1],pins[1][1]), GPIO.LOW)
        sleep(step)
        GPIO.output((pins[0][0],pins[1][0]), GPIO.LOW)
        GPIO.output((pins[1][1],pins[1][1]), GPIO.LOW)

def distance(pins):
    """ Distance measurement with ultrasonic sensor
    Arg:
        pins: array of size 2 [trigger, echo]
    Return:
        distance
    """

    # set Trigger to HIGH
    GPIO.output(pins[0], True)

    # set Trigger after 0.01ms to LOW
    time.sleep(0.00001)
    GPIO.output(pins[0], False)

    StartTime = time.time()
    StopTime = time.time()

    # save StartTime
    while GPIO.input(pins[1]) == 0:
        StartTime = time.time()

    # save time of arrival
    while GPIO.input(pins[1]) == 1:
        StopTime = time.time()

    # time difference between start and arrival
    TimeElapsed = StopTime - StartTime
    # multiply with the sonic speed (34300 cm/s)
    # and divide by 2, because there and back
    distance = (TimeElapsed * 34300) / 2

    return distance

def isxnan (a):
    """ Mark cells with NaN as False

    Args:
        a: an array
    """
    a = np.isnan(a)
    for i in range(a.size):
        if a[i] == False:
            a[i] = True
        else:
            a[i] = False
    
    return a