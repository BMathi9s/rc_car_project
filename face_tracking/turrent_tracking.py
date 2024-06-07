import os
import cv2
from ultralytics import YOLO
from supervision import Detections
from PIL import Image
import numpy as np
from centroidtracker import CentroidTracker
from turret import Turret

def main():
    # Define the local path to the model file
    local_model_path = "../models--arnabdhar--YOLOv8-Face-Detection/snapshots/52fa54977207fa4f021de949b515fb19dcab4488/model.pt"
    
    # Check if the local model file exists
    if not os.path.exists(local_model_path):
        print(f"Model file not found at {local_model_path}. Please download it first.")
        return

    # Load model
    model = YOLO(local_model_path)

    # Open the default camera (usually the first one)
    cap = cv2.VideoCapture(0)
    # Check if the camera opened successfully
    if not cap.isOpened():
        print("Error: Could not open camera.")
        return

    # Initialize the turret with appropriate servo channels
    turret = Turret(base_channel=0, canon_channel=1)
    turret.set_proportionnal_constant(0.01)

    # Initialize the CentroidTracker for face tracking
    centroid_tracker = CentroidTracker(maxDisappeared=50, maxDistance=50)

    while True:
        # Capture frame-by-frame
        ret, frame = cap.read()
        if not ret:
            print("Error: Failed to capture image.")
            break

        # Convert the image from BGR color (which OpenCV uses) to RGB color
        rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

        # Perform inference on the RGB image
        output = model(Image.fromarray(rgb_frame))
        results = Detections.from_ultralytics(output[0])

        rects = []  # List to store detected face bounding boxes

        # Iterate over the detected faces
        for result in results.xyxy:
            x1, y1, x2, y2 = result  # Extract coordinates
            rects.append([x1, y1, x2, y2])
            # Draw rectangle around the face
            cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), (0, 0, 255), 2)

        # Update the CentroidTracker with the detected face bounding boxes
        objects = centroid_tracker.update(rects)

        # Select the closest face to the center of the frame
        closest_face = None
        closest_distance = float('inf')
        frame_center = (frame.shape[1] // 2, frame.shape[0] // 2)

        for (objectID, bbox) in objects.items():
            x1, y1, x2, y2 = bbox.astype("int")
            face_center = ((x1 + x2) // 2, (y1 + y2) // 2)
            distance = np.linalg.norm(np.array(frame_center) - np.array(face_center))

            if distance < closest_distance:
                closest_distance = distance
                closest_face = face_center

        if closest_face:
            # Calculate the difference between the face center and the frame center
            x_diff = closest_face[0] - frame_center[0]
            y_diff = closest_face[1] - frame_center[1]

            # Update the turret position
            turret.update(x_diff, y_diff)
            cv2.circle(frame, closest_face, 5, (0, 255, 0), -1)  # Draw a circle at the center of the closest face

        # Display the resulting frame
        cv2.imshow('Camera', frame)

        # Break the loop if 'q' is pressed
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # Release the camera and close the window
    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
