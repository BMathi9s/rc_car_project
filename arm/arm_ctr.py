# load libraries
import cv2
import os
from ultralytics import YOLO
from supervision import Detections
from PIL import Image
import numpy as np
from turret import Turret


def main():
    # download model
    # model_path = hf_hub_download(repo_id="arnabdhar/YOLOv8-Face-Detection", filename="model.pt")
    
    # load model
    # model = YOLO(model_path)
    # Open the default camera (usually the first one)
    
    local_model_path = "models--arnabdhar--YOLOv8-Face-Detection/snapshots/52fa54977207fa4f021de949b515fb19dcab4488/model.pt"
    
    # Check if the local model file exists
    if not os.path.exists(local_model_path):
        print(f"Model file not found at {local_model_path}. Please download it first.")
        return

    # Load model
    model = YOLO(local_model_path)
    
    
    cap = cv2.VideoCapture(0)
    # Check if the camera opened successfully
    if not cap.isOpened():
        print("Error: Could not open camera.")
        return
    
    # Initialize the turret
    turret = Turret(base_channel=0, canon_channel=1)
    turret.set_proportionnal_constant(constant=0.05)  # Set the proportional constant
    #turret.set_damping_factor(damping_factor=0.5)  # Set the damping factor
    
    while True:
        # Capture frame-by-frame
        ret, frame = cap.read()

        # Convert the image from BGR color (which OpenCV uses) to RGB color
        rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

        # Perform inference on the RGB image
        output = model(Image.fromarray(rgb_frame))
        results = Detections.from_ultralytics(output[0])
        
        # Calculate the center of the frame
        frame_height, frame_width = frame.shape[:2]
        center_x, center_y = frame_width // 2, frame_height // 2
        
        min_distance = float('inf')
        closest_face_coords = None
        face_diff_x = 0
        face_diff_y = 0
        
        # Check all detected faces and find the closest one to the center
        for result in results.xyxy:
            x1, y1, x2, y2 = result
            face_center_x = (x1 + x2) // 2
            face_center_y = (y1 + y2) // 2
            distance = np.sqrt((face_center_x - center_x) ** 2 + (face_center_y - center_y) ** 2)
            
            if distance < min_distance:
                min_distance = distance
                closest_face_coords = (x1, y1, x2, y2)
                face_diff_x = face_center_x - center_x
                face_diff_y = face_center_y - center_y
        
        if closest_face_coords:
            x1, y1, x2, y2 = closest_face_coords
            print(f"Closest face detected at x1={x1}, y1={y1}, x2={x2}, y2={y2}")
            print(f"Difference from center: x_diff={face_diff_x}, y_diff={face_diff_y}")
            # Draw rectangle around the closest face
            # cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), (0, 0, 255), 2)
            
            # Only update turret if difference is significant
            if abs(face_diff_x) > 10 or abs(face_diff_y) > 10:
                # Update turret position
                turret.update(face_diff_x, face_diff_y)
        
        # Display the resulting frame
        # cv2.imshow('Camera', frame)

        # Break the loop if any key is pressed
        # if cv2.waitKey(1) != -1:
        #     break

    # Release the camera and close the window
    # cap.release()
    # cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
