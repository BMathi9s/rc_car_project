import cv2
import numpy as np
from mediapipe import solutions
from mediapipe.framework.formats import landmark_pb2
import mediapipe as mp
from mediapipe.tasks import python
from mediapipe.tasks.python import vision

def draw_landmarks_on_image(rgb_image, detection_result, closest_person_idx):
    pose_landmarks_list = detection_result.pose_landmarks
    annotated_image = np.copy(rgb_image)

    colors = [
        (255, 0, 0),  # Red
        (0, 255, 0),  # Green
        (0, 0, 255),  # Blue
        (255, 255, 0),  # Cyan
        (255, 0, 255)  # Magenta
    ]

    closest_color = (0, 0, 0)  # Black for the closest person

    # Loop through the detected poses to visualize.
    for idx in range(min(len(pose_landmarks_list), 5)):
        pose_landmarks = pose_landmarks_list[idx]
        color = closest_color if idx == closest_person_idx else colors[idx % len(colors)]

        # Draw the pose landmarks.
        pose_landmarks_proto = landmark_pb2.NormalizedLandmarkList()
        pose_landmarks_proto.landmark.extend([
            landmark_pb2.NormalizedLandmark(x=landmark.x, y=landmark.y, z=landmark.z) for landmark in pose_landmarks
        ])
        solutions.drawing_utils.draw_landmarks(
            annotated_image,
            pose_landmarks_proto,
            solutions.pose.POSE_CONNECTIONS,
            landmark_drawing_spec=solutions.drawing_styles.DrawingSpec(color=color, thickness=2, circle_radius=2),
            connection_drawing_spec=solutions.drawing_styles.DrawingSpec(color=color, thickness=2))
    return annotated_image

def find_closest_person(pose_landmarks_list):
    min_z = float('inf')
    closest_idx = -1

    for idx, pose_landmarks in enumerate(pose_landmarks_list):
        # Calculate the average Z coordinate of the landmarks to determine proximity
        avg_z = np.mean([landmark.z for landmark in pose_landmarks])
        if avg_z < min_z:
            min_z = avg_z
            closest_idx = idx

    return closest_idx

def main():

    # STEP 1: Create a PoseLandmarker object.
    base_options = python.BaseOptions(model_asset_path='pose_landmarker_lite.task')
    options = vision.PoseLandmarkerOptions(
        base_options=base_options,
        output_segmentation_masks=True,
        num_poses=5  # Set to detect up to 5 poses
    )
    detector = vision.PoseLandmarker.create_from_options(options)

    # STEP 2: Capture video from the webcam.
    cap = cv2.VideoCapture(0)  # Change to 'video_file.mp4' for a video file


    while cap.isOpened():
        ret, frame = cap.read()
        if not ret:
            break


        # Convert the frame to RGB as Mediapipe expects RGB input
        rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

        # Create a MediaPipe Image object from the frame
        image = mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb_frame)

        # STEP 3: Detect pose landmarks from the input frame.
        detection_result = detector.detect(image)

        # STEP 4: Find the closest person
        closest_person_idx = find_closest_person(detection_result.pose_landmarks)

        # STEP 5: Process the detection result. In this case, visualize it.
        annotated_image = draw_landmarks_on_image(rgb_frame, detection_result, closest_person_idx)
        pose_landmarks = detection_result.pose_landmarks[closest_person_idx]
        nose = pose_landmarks[0]
        nose_x = int(nose.x * frame.shape[1])
        nose_y = int(nose.y * frame.shape[0])

        # Draw a circle on the nose
        cv2.circle(annotated_image, (nose_x, nose_y), 5, (0, 255, 0), -1)
        
        cv2.imshow("Annotated Video", cv2.cvtColor(annotated_image, cv2.COLOR_RGB2BGR))

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
