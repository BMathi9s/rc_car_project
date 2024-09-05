# main.py
from motor_driver import MotorDriver
import time

def motor_test():
    motor1 = MotorDriver(17, 18)  # create a MotorDriver object with dir_pin=17 and pwm_pin=18
    motor2 = MotorDriver(22, 27)

    try:
        for speed in range(0, 101):
            motor1.set_speed(speed/100)  # gradually increase speed from 0 to 1
            motor2.set_speed(speed/100)
            time.sleep(0.2)  # wait for 0.1 seconds before next change

    except KeyboardInterrupt:  # trap a CTRL+C keyboard interrupt
        motor1.set_speed(0)  # stop motor
        motor2.set_speed(0)

    finally:
        motor1.close()  # in case of unexpected exit, resets pin status (motor will go full speed after exiting)
        motor2.close()

if __name__ == "__main__":
    motor_test()
