import cv2
import mediapipe as mp
import time

from Marshmellow_Cannon import Marshmellow_Cannon
# Example usage


# Initialize MediaPipe Pose
mp_pose = mp.solutions.pose
pose = mp_pose.Pose()
mp_drawing = mp.solutions.drawing_utils

# Initialize video capture with lower resolution
cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

# Set frame rate limit
fps_limit = 30
prev_time = 0
center_of_screen = 0.5

#ini cannon
cannon = Marshmellow_Cannon(base_channel=0, cannon_channel=1)
cannon.center()
cannon.set_camera_scope(10)

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
            
            print(f'Nose coordinates: {nose}')
             # Update turret position
            face_diff_x = nose.x - center_of_screen
            face_diff_y = nose.y - center_of_screen
            
            print(f'dif x: {face_diff_x}')
            print(f'dif y: {face_diff_y}')
            
            if 0.4 <= nose.x <= 0.6 and 0.4 <= nose.y <= 0.6:
              cannon.track_face(nose.x, nose.y)  # Adjust servos to track the face

        # Display the frame
        cv2.imshow('MediaPipe Pose', image)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

cap.release()
cv2.destroyAllWindows()