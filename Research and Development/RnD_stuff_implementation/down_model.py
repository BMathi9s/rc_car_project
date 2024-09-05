# load libraries
import cv2
import os
from ultralytics import YOLO
from supervision import Detections
from PIL import Image

def main():
    # Define the local path to the model file
    local_model_path = "models--arnabdhar--YOLOv8-Face-Detection/snapshots/52fa54977207fa4f021de949b515fb19dcab4488/model.pt"
    
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

        # Print the bounding box coordinates of each detected face
        for result in results.xyxy:
            x1, y1, x2, y2 = result  # Extract coordinates
            print(f"Face detected at x1={x1}, y1={y1}, x2={x2}, y2={y2}")
            # Draw rectangle around the face
            cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), (0, 0, 255), 2)

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
