# tracking.py
import cv2
import mediapipe as mp
from turret import Turret

# Initialize MediaPipe Pose
mp_pose = mp.solutions.pose
pose = mp_pose.Pose()
mp_drawing = mp.solutions.drawing_utils

# Camera setup
cap = cv2.VideoCapture(0)
if not cap.isOpened():
    print("Error: Could not open video device.")
    exit()

# Initialize Turret
horizontal_fov = 60
vertical_fov = 45
turret = Turret(horizontal_channel=0, vertical_channel=1, horizontal_fov=horizontal_fov, vertical_fov=vertical_fov, easing_factor=0.1, dead_zone=10)

while cap.isOpened():
    ret, frame = cap.read()
    if not ret:
        break

    # Convert the BGR image to RGB
    image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    # Process the image and find poses
    results = pose.process(image)

    # Draw the pose annotation on the image.
    image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
    if results.pose_landmarks:
        mp_drawing.draw_landmarks(image, results.pose_landmarks, mp_pose.POSE_CONNECTIONS)

        # Get the coordinates of the nose (as an example)
        nose = results.pose_landmarks.landmark[mp_pose.PoseLandmark.NOSE]
        x, y = int(nose.x * frame.shape[1]), int(nose.y * frame.shape[0])

        # Draw a circle at the nose position
        cv2.circle(image, (x, y), 10, (0, 255, 0), -1)

        # Update turret position based on nose coordinates
        turret.update_position(x, y, frame.shape[1], frame.shape[0])

    cv2.imshow('Frame', image)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
