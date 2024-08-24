import numpy as np

class Marshmellow_Cannon:
    def __init__(self, base_channel, cannon_channel, base_angle=90, cannon_angle=90, function_type_x='exponential', function_type_y='exponential'):
        self.kit = ServoKit(channels=16)
        self.base_channel = base_channel
        self.cannon_channel = cannon_channel
        self.base_angle = base_angle
        self.cannon_angle = cannon_angle
        self.kit.servo[self.base_channel].angle = self.base_angle
        self.kit.servo[self.cannon_channel].angle = self.cannon_angle
        self.camera_scope = 90
        self.function_type_x = function_type_x
        self.function_type_y = function_type_y
        
    def set_camera_scope(self, scope):
        self.camera_scope = scope

    def set_function_types(self, function_type_x, function_type_y):
        self.function_type_x = function_type_x
        self.function_type_y = function_type_y
        
    def set_angles(self, base_angle, cannon_angle):
        self.base_angle = base_angle
        self.cannon_angle = cannon_angle
        self.kit.servo[self.base_channel].angle = self.base_angle
        self.kit.servo[self.cannon_channel].angle = self.cannon_angle

    def center(self):
        self.set_angles(90, 135)

    def apply_function(self, diff, function_type):
        # Normalize diff to the range [0, 1] based on the maximum possible diff (0.5)
        normalized_diff = diff / 0.5

        if function_type == 'linear':
            return self.camera_scope * normalized_diff
        elif function_type == 'quadratic':
            return self.camera_scope * (normalized_diff ** 2)
        elif function_type == 'cubic':
            return self.camera_scope * (normalized_diff ** 3)
        elif function_type == 'exponential':
            return self.camera_scope * (np.exp(normalized_diff) - 1) / (np.exp(1) - 1)
        elif function_type == 'logarithmic':
            return self.camera_scope * np.log1p(normalized_diff) / np.log1p(1)
        elif function_type == 'sine':
            return self.camera_scope * np.sin(normalized_diff * np.pi / 2)
        elif function_type == 'inverse_quadratic':
            return self.camera_scope * (1 - (1 - normalized_diff) ** 2)
        elif function_type == 'sigmoid':
            k = 10  # steepness factor
            return self.camera_scope * (1 / (1 + np.exp(-k * (normalized_diff - 0.5))))
        else:
            raise ValueError("Unsupported function type provided.")

    def track_face(self, x, y, deltax):
        x_diff = x - 0.5 
        y_diff = y - 0.5
        
        # Apply the selected functions separately to x_diff and y_diff
        dynamic_scope_x = self.apply_function(abs(x_diff), self.function_type_x)
        dynamic_scope_y = self.apply_function(abs(y_diff), self.function_type_y)
        
        # Adjust angles with the separate dynamic scopes
        base_angle = self.base_angle + (-x_diff * dynamic_scope_x)
        cannon_angle = self.cannon_angle + (y_diff * dynamic_scope_y)

        # Ensure the angles are within the valid range
        base_angle = max(0, min(180, base_angle))
        cannon_angle = max(0, min(180, cannon_angle))

        # Set the new angles
        self.set_angles(base_angle, cannon_angle)
