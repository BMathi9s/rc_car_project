import cv2
import os
import time
from ultralytics import YOLO
from supervision import Detections
from PIL import Image
import numpy as np
from turret import Turret

def main():
    local_model_path = "../models--arnabdhar--YOLOv8-Face-Detection/snapshots/52fa54977207fa4f021de949b515fb19dcab4488/model.pt"
    
    if not os.path.exists(local_model_path):
        print(f"../Model file not found at {local_model_path}. Please download it first.")
        return

    model = YOLO(local_model_path)
    
    cap = cv2.VideoCapture(0)
    
    if not cap.isOpened():
        print("Error: Could not open camera.")
        return
    
    # Initialize the turret
    turret = Turret(base_channel=0, canon_channel=1)
    turret.set_proportionnal_constant(constant=0.02)  # Set the proportional constant

    # Choose your tracker here
    tracker = cv2.TrackerMIL.create()
    
    tracking = False
    last_inference_time = time.time()  # Track the time of the last inference
    
    while True:
        ret, frame = cap.read()
        if not ret:
            break

        rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

        # Check if 10 seconds have passed since the last inference
        current_time = time.time()
        if tracking and (current_time - last_inference_time) > 10:
            tracking = False  # Force a new inference

        if not tracking:
            output = model(Image.fromarray(rgb_frame))
            results = Detections.from_ultralytics(output[0])
            
            max_area = 0
            closest_face_coords = None
            
            for result in results.xyxy:
                x1, y1, x2, y2 = result
                width = x2 - x1
                height = y2 - y1
                area = width * height
                
                if area > max_area:
                    max_area = area
                    closest_face_coords = (x1, y1, x2, y2)
                    face_center_x = (x1 + x2) // 2
                    face_center_y = (y1 + y2) // 2
                    
            if closest_face_coords:
                x1, y1, x2, y2 = closest_face_coords
                print(f"Closest face detected at x1={x1}, y1={y1}, x2={x2}, y2={y2}")
                print(f"Center of face: x={face_center_x}, y={face_center_y}")
                cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), (0, 0, 255), 2)
                
                bbox = (int(x1), int(y1), int(x2-x1), int(y2-y1))
                tracker.init(frame, bbox)
                tracking = True
                last_inference_time = current_time  # Reset the inference timer
        else:
            tracking, bbox = tracker.update(frame)
            if tracking:
                p1 = (int(bbox[0]), int(bbox[1]))
                p2 = (int(bbox[0] + bbox[2]), int(bbox[1] + bbox[3]))
                face_center_x = p1[0] + (bbox[2] // 2)
                face_center_y = p1[1] + (bbox[3] // 2)
                print(f"Tracking face center: x={face_center_x}, y={face_center_y}")
                cv2.rectangle(frame, p1, p2, (255, 0, 0), 2, 1)
                
                # Calculate the difference from the center of the screen
                screen_center_x = frame.shape[1] // 2
                screen_center_y = frame.shape[0] // 2
                face_diff_x = face_center_x - screen_center_x
                face_diff_y = face_center_y - screen_center_y
                print(f"Tracking face center differences : x={face_diff_x}, y={face_diff_y}")
                
                if abs(face_diff_x) > 20 or abs(face_diff_y) > 20:
                    # Update turret position
                    turret.update(face_diff_x, face_diff_y)
            else:
                tracking = False

        # cv2.imshow('Camera', frame)

        # if cv2.waitKey(1) != -1:
        #     break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
