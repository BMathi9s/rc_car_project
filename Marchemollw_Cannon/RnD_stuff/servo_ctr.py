from adafruit_servokit import ServoKit
import time

# Initialize PCA9685 with 16 channels
kit = ServoKit(channels=16)

# Control a servo on channel 0
while True:
    # Move servo to 0 degrees
    kit.servo[0].angle = 90
    time.sleep(1)
    kit.servo[1].angle = 100
    time.sleep(1)

  
