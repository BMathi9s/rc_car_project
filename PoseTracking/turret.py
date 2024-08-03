# turret.py
from adafruit_servokit import ServoKit

class Turret:
    def __init__(self, horizontal_channel, vertical_channel, horizontal_fov, vertical_fov, easing_factor=0.1, dead_zone=10):
        self.kit = ServoKit(channels=16)
        self.horizontal_channel = horizontal_channel
        self.vertical_channel = vertical_channel
        self.horizontal_fov = horizontal_fov
        self.vertical_fov = vertical_fov
        self.easing_factor = easing_factor
        self.dead_zone = dead_zone

        # Initial servo positions
        self.x_servo_pos = 90
        self.y_servo_pos = 90

    def pixel_to_angle(self, pixel, frame_size, fov):
        center = frame_size / 2
        return ((pixel - center) / center) * (fov / 2)

    def update_position(self, x, y, frame_width, frame_height):
        # Calculate angles relative to the camera's FOV
        x_angle = self.pixel_to_angle(x, frame_width, self.horizontal_fov)
        y_angle = self.pixel_to_angle(y, frame_height, self.vertical_fov)

        # Check if the detected point is outside the dead zone
        if abs(x - frame_width / 2) > self.dead_zone or abs(y - frame_height / 2) > self.dead_zone:
            # Calculate the target servo positions with easing
            target_x_servo_pos = self.x_servo_pos + (x_angle * self.easing_factor)
            target_y_servo_pos = self.y_servo_pos + (y_angle * self.easing_factor)

            # Ensure servo angles are within bounds (0-180 degrees)
            self.x_servo_pos = max(0, min(180, target_x_servo_pos))
            self.y_servo_pos = max(0, min(180, target_y_servo_pos))

            # Move servos
            self.kit.servo[self.horizontal_channel].angle = self.x_servo_pos
            self.kit.servo[self.vertical_channel].angle = self.y_servo_pos
