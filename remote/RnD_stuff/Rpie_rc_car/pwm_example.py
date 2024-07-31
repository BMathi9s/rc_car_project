#! /usr/bin/env python3
from gpiozero import PWMOutputDevice, DigitalOutputDevice
import time

# Configuration
DIRECTION_PIN = 17  # BCM pin used to set motor direction
SPEED_PIN = 18      # BCM pin used to control motor speed
WAIT_TIME = 2       # [s]8Time to wait between each refresh
PWM_FREQ = 10       # [kHz] 25kHz for PWM control

# Configurable motor speed and direction
MOTOR_MIN = 0       # minimum motor speed (0 = stop)
MOTOR_MAX = 1       # maximum motor speed (1 = full speed)

# Set motor speed
def setMotorSpeed(speed):
    pwm_motor.value = speed  # set speed from 0 to 1
    return()

# Set motor direction
def setMotorDirection(direction):
    dir_motor.value = direction  # set direction (0 = backward, 1 = forward)
    return()

try:
    pwm_motor = PWMOutputDevice(SPEED_PIN, initial_value=0, frequency=PWM_FREQ)  # initialize SPEED_PIN as a pwm output
    dir_motor = DigitalOutputDevice(DIRECTION_PIN)  # initialize DIRECTION_PIN as a digital output
    setMotorSpeed(MOTOR_MIN)  # initially set motor speed to the MOTOR_MIN value
    setMotorDirection(1)  # initially set motor direction to forward

    while True:
        for speed in range(0, 11):
            setMotorSpeed(speed/10)  # gradually increase speed from 0 to 1
            time.sleep(WAIT_TIME)  # wait for WAIT_TIME seconds before next change

        setMotorDirection(0 if dir_motor.value else 1)  # switch direction

except KeyboardInterrupt:  # trap a CTRL+C keyboard interrupt
    setMotorSpeed(MOTOR_MIN)

finally:
    pwm_motor.close()  # in case of unexpected exit, resets pin status (motor will go full speed after exiting)
    dir_motor.close()  # resets pin status
