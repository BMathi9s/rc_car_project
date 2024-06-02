# Import necessary libraries
import cv2
import os
from ultralytics import YOLO
from supervision import Detections
from PIL import Image
from turret import Turret

# Main function
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
    
    # Initialize turret
    turret = Turret(base_channel=0, canon_channel=1)

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

        # Find the closest face
        closest_face = None
        max_area = 0
        for result in results.xyxy:
            x1, y1, x2, y2 = result  # Extract coordinates
            area = (x2 - x1) * (y2 - y1)
            if area > max_area:
                max_area = area
                closest_face = result

        if closest_face is not None:
            x1, y1, x2, y2 = closest_face
            # Calculate center of the face
            face_center_x = (x1 + x2) / 2
            face_center_y = (y1 + y2) / 2

            # Calculate the difference from the center of the frame
            frame_center_x = frame.shape[1] / 2
            frame_center_y = frame.shape[0] / 2
            x_diff = face_center_x - frame_center_x
            y_diff = face_center_y - frame_center_y

            # Update the turret to center the face
            turret.update(x_diff, y_diff)

            # Draw rectangle around the closest face
            cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), (0, 0, 255), 2)
        else:
            # No face detected, stop the turret movement
            turret.update(0, 0)

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
