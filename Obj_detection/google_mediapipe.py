import cv2
import mediapipe as mp
import time

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
            nose_x = int(nose.x * image.shape[1])
            nose_y = int(nose.y * image.shape[0])

            # Draw a circle on the nose
            cv2.circle(image, (nose_x, nose_y), 5, (0, 255, 0), -1)
            
            # Print the coordinates
            print(f'Image shape: {image.shape}')
            print(f'Nose coordinates: x={nose_x}, y={nose_y}')
            print(f'Nose coordinates not scaled: {nose}')

        # Display the frame
        cv2.imshow('MediaPipe Pose', image)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

cap.release()
cv2.destroyAllWindows()