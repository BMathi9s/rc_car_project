# motor_driver.py
from gpiozero import PWMOutputDevice, DigitalOutputDevice

class MotorDriver:
    def __init__(self, dir_pin, pwm_pin, pwm_freq=10):
        self.dir_motor = DigitalOutputDevice(dir_pin)
        self.pwm_motor = PWMOutputDevice(pwm_pin, initial_value=0, frequency=pwm_freq)

    def set_speed(self, speed):
        self.pwm_motor.value = speed

    def set_dir(self, direction):
        self.dir_motor.value = direction

    def close(self):
        self.pwm_motor.close()
        self.dir_motor.close()
