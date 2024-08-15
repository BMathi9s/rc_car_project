# Necessary imports
from adafruit_servokit import ServoKit
import numpy as np
import cv2
import mediapipe as mp
import time

class Marshmellow_Cannon:
    def __init__(self, base_channel, cannon_channel, base_angle=90, cannon_angle=90, deadzone=5, max_delta=10, speed=1):
        self.kit = ServoKit(channels=16)
        self.base_channel = base_channel
        self.cannon_channel = cannon_channel
        self.base_angle = base_angle
        self.cannon_angle = cannon_angle
        self.deadzone = deadzone  # Deadzone in degrees
        self.max_delta = max_delta  # Maximum change to prevent overshoot
        self.speed = speed  # Speed factor for movement
        self.camera_scope = 90

        # Caps for each direction (120 degrees max in each direction from 90 degrees)
        self.base_min_angle = 90 - 120 // 2
        self.base_max_angle = 90 + 120 // 2
        self.cannon_min_angle = 90 - 120 // 2
        self.cannon_max_angle = 90 + 120 // 2

        self.kit.servo[self.base_channel].angle = self.base_angle
        self.kit.servo[self.cannon_channel].angle = self.cannon_angle

    def move_base(self, angle_delta):
        # Apply speed factor and limit to avoid overshooting
        angle_delta = np.clip(angle_delta * self.speed, -self.max_delta, self.max_delta)
        self.base_angle += angle_delta
        # Apply caps to the base angle
        self.base_angle = np.clip(self.base_angle, self.base_min_angle, self.base_max_angle)
        self.kit.servo[self.base_channel].angle = self.base_angle

    def move_cannon(self, angle_delta):
        # Apply speed factor and limit to avoid overshooting
        angle_delta = np.clip(angle_delta * self.speed, -self.max_delta, self.max_delta)
        self.cannon_angle += angle_delta
        # Apply caps to the cannon angle
        self.cannon_angle = np.clip(self.cannon_angle, self.cannon_min_angle, self.cannon_max_angle)
        self.kit.servo[self.cannon_channel].angle = self.cannon_angle

    def track_target(self, nose_x, nose_y, frame_width, frame_height):
        # Calculate the horizontal angle offset
        x_center_offset = (nose_x - frame_width * 0.5) / frame_width
        horizontal_angle_offset = x_center_offset * self.camera_scope

        # Calculate the vertical angle offset
        y_center_offset = (nose_y - frame_height * 0.5) / frame_height
        vertical_angle_offset = y_center_offset * self.camera_scope

        # Apply deadzone to avoid small unnecessary adjustments
        if abs(horizontal_angle_offset) > self.deadzone:
            self.move_base(horizontal_angle_offset)

        if abs(vertical_angle_offset) > self.deadzone:
            self.move_cannon(-vertical_angle_offset)  # Invert for correct vertical tracking

    def reset_position(self):
        # Reset base and cannon to initial position
        self.base_angle = 90
        self.cannon_angle = 90
        self.kit.servo[self.base_channel].angle = self.base_angle
        self.kit.servo[self.cannon_channel].angle = self.cannon_angle

# Initialize the turret with deadzone, max delta, and speed
turret = Marshmellow_Cannon(base_channel=0, cannon_channel=1, deadzone=5, max_delta=10, speed=1)

# Initialize MediaPipe Pose
mp_pose = mp.solutions.pose
pose = mp_pose.Pose()
mp_drawing = mp.solutions.drawing_utils

# Initialize video capture with lower resolution
cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

# Set frame rate limit
fps_limit = 90
prev_time = 0
center_of_screen = 0.5

while cap.isOpened():
    time_elapsed = time.time() - prev_time
    if time_elapsed > 1./fps_limit:
        prev_time = time.time()

        ret, frame = cap.read()
        if not ret:
            break

        # Convert the frame to RGB
        image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

        # Process the frame and find pose
        results = pose.process(image)

        # Convert back to BGR for OpenCV
        image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)

        # Draw the pose annotation on the image and get the nose coordinates
        if results.pose_landmarks:
            mp_drawing.draw_landmarks(image, results.pose_landmarks, mp_pose.POSE_CONNECTIONS)
            
            # Get the nose coordinates
            nose = results.pose_landmarks.landmark[0]
            nose_x = int(nose.x * frame.shape[1])
            nose_y = int(nose.y * frame.shape[0])
            
            # Track the target with overshoot protection, deadzone, and speed control
            turret.track_target(nose_x, nose_y, frame.shape[1], frame.shape[0])

        # Display the frame with annotations
        cv2.imshow('Turret Tracking', image)

        # Break the loop on 'q' key press
        if cv2.waitKey(1) & 0xFF == ord('q'):
            turret.reset_position()
            break

cap.release()
cv2.destroyAllWindows()
