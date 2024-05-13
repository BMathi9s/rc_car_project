from adafruit_servokit import ServoKit
import time

# Initialize PCA9685 with 16 channels
kit = ServoKit(channels=16)

# Control a servo on channel 0
while True:
    # Move servo to 0 degrees
    kit.servo[0].angle = 0
    time.sleep(1)

    # Move servo to 180 degrees
    kit.servo[0].angle = 180
    time.sleep(1)
