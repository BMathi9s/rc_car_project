from adafruit_servokit import ServoKit

class Turret:
    def __init__(self, base_channel, canon_channel, min_angle=0, max_angle=180):
        self.kit = ServoKit(channels=16)
        self.base_channel = base_channel
        self.canon_channel = canon_channel
        self.min_angle = min_angle
        self.max_angle = max_angle
        self.base_angle = (min_angle + max_angle) // 2
        self.canon_angle = (min_angle + max_angle) // 2
        self.kit.servo[self.base_channel].angle = self.base_angle
        self.kit.servo[self.canon_channel].angle = self.canon_angle
        self.prop_const = 0.01

    def update(self, x_diff, y_diff):
        # Here, you need to convert x_diff and y_diff into angle changes.
        # This example assumes a simple proportional control.
        base_change = -x_diff * self.prop_const  # Adjust the factor as needed
        canon_change = -y_diff * self.prop_const  # Adjust the factor as needed

        self.base_angle = max(self.min_angle, min(self.max_angle, self.base_angle + base_change))
        self.canon_angle = max(self.min_angle, min(self.max_angle, self.canon_angle + canon_change))
        
        print(f"turret ctr:  base ={self.base_angle}, canon={self.canon_angle}")

        self.kit.servo[self.base_channel].angle = self.base_angle
        self.kit.servo[self.canon_channel].angle = self.canon_angle
    
    def set_proportionnal_constant(self, constant):
        self.prop_const = constant
