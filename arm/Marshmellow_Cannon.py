from adafruit_servokit import ServoKit

class Marshmellow_Cannon:
    def __init__(self, base_channel, cannon_channel, base_angle=120, cannon_angle=45):
        self.kit = ServoKit(channels=16)
        self.base_channel = base_channel
        self.cannon_channel = cannon_channel
        self.base_angle = base_angle
        self.cannon_angle = cannon_angle
        self.kit.servo[self.base_channel].angle = self.base_angle
        self.kit.servo[self.cannon_channel].angle = self.cannon_angle
        self.camera_scope = 90
    
    def set_camera_scope(self, scope):
        #Center the servos at 90 degrees.
        self.camera_scope = scope
        
        
    def set_angles(self, base_angle, cannon_angle):
        #Set the base and cannon angles to specified values.
        self.base_angle = base_angle
        self.cannon_angle = cannon_angle
        self.kit.servo[self.base_channel].angle = self.base_angle
        self.kit.servo[self.cannon_channel].angle = self.cannon_angle

    def center(self):
        #Center the servos at 90 degrees.
        self.set_angles(90, 90)

    def track_face(self, x, y):
        #Adjust the servos based on the normalized x and y coordinates of the detected face.
        # Calculate the difference from the center
        x_diff = x - 0.5 
        y_diff = y - 0.5

        # Calculate the new angles based on the difference
        base_angle = self.base_angle + (-x_diff * self.camera_scope)  # Scaling the difference to the servo angle range
        cannon_angle = self.cannon_angle + (y_diff * self.camera_scope)

        # Ensure the angles are within the valid range
        base_angle = max(0, min(180, base_angle))
        cannon_angle = max(0, min(180, cannon_angle))

        # Set the new angles
        self.set_angles(base_angle, cannon_angle)


