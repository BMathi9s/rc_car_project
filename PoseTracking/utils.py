
import numpy as np
from mediapipe import solutions
from mediapipe.framework.formats import landmark_pb2

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
