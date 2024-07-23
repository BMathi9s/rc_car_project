
import cv2
import numpy as np
from mediapipe import solutions
from mediapipe.framework.formats import landmark_pb2
import mediapipe as mp
from mediapipe.tasks import python
from mediapipe.tasks.python import vision
from turret import Turret
from utils import draw_landmarks_on_image, find_closest_person

def main():
    # STEP 1: Import the necessary modules.
    # Already done at the top

    # STEP 2: Create a PoseLandmarker object.
    base_options = python.BaseOptions(model_asset_path='pose_landmarker_lite.task')
    options = vision.PoseLandmarkerOptions(
        base_options=base_options,
        output_segmentation_masks=True,
        num_poses=5  # Set to detect up to 5 poses
    )
    detector = vision.PoseLandmarker.create_from_options(options)

    # STEP 3: Initialize the turret
    turret = Turret(base_channel=0, canon_channel=1)

    # STEP 4: Capture video from the webcam.
    cap = cv2.VideoCapture(0)  # Change to 'video_file.mp4' for a video file

    window_size_limit = 1270  # Set the window size limit

    while cap.isOpened():
        ret, frame = cap.read()
        if not ret:
            break

        # Resize the frame to fit within the window size limit
        height, width = frame.shape[:2]
        if width > window_size_limit:
            scaling_factor = window_size_limit / width
            frame = cv2.resize(frame, (int(width * scaling_factor), int(height * scaling_factor)))

        # Convert the frame to RGB as Mediapipe expects RGB input
        rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

        # Create a MediaPipe Image object from the frame
        image = mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb_frame)

        # STEP 5: Detect pose landmarks from the input frame.
        detection_result = detector.detect(image)

        # STEP 6: Find the closest person
        closest_person_idx = find_closest_person(detection_result.pose_landmarks)

        if closest_person_idx != -1:
            # Get the nose point of the closest person
            pose_landmarks = detection_result.pose_landmarks[closest_person_idx]
            nose = pose_landmarks[0]
            nose_x = int(nose.x * image.shape[1])
            nose_y = int(nose.y * image.shape[0])

            # Calculate the difference from the center of the frame
            frame_center_x = width // 2
            frame_center_y = height // 2
            x_diff = nose_x - frame_center_x
            y_diff = nose_y - frame_center_y

            # Update the turret based on the difference
            turret.update(x_diff, y_diff)

        # STEP 7: Process the detection result. In this case, visualize it.
        annotated_image = draw_landmarks_on_image(rgb_frame, detection_result, closest_person_idx)
        cv2.imshow("Annotated Video", cv2.cvtColor(annotated_image, cv2.COLOR_RGB2BGR))

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
