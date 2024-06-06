import cv2
import os
from ultralytics import YOLO
from supervision import Detections
from PIL import Image
import numpy as np
from centroidtracker import CentroidTracker  # Make sure to save the CentroidTracker class in centroid_tracker.py

def main():
    local_model_path = "models--arnabdhar--YOLOv8-Face-Detection/snapshots/52fa54977207fa4f021de949b515fb19dcab4488/model.pt"
    
    if not os.path.exists(local_model_path):
        print(f"Model file not found at {local_model_path}. Please download it first.")
        return

    model = YOLO(local_model_path)
    
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("Error: Could not open camera.")
        return

    # Initialize the CentroidTracker
    ct = CentroidTracker(maxDisappeared=50, maxDistance=50)
    tracking = False
    tracked_object_id = None

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

        if not tracking:
            output = model(Image.fromarray(rgb_frame))
            results = Detections.from_ultralytics(output[0])
            
            max_area = 0
            closest_face_coords = None
            
            rects = []
            for result in results.xyxy:
                x1, y1, x2, y2 = map(int, result)
                width = x2 - x1
                height = y2 - y1
                area = width * height
                
                if area > max_area:
                    max_area = area
                    closest_face_coords = (x1, y1, x2, y2)
                    face_center_x = (x1 + x2) // 2
                    face_center_y = (y1 + y2) // 2
                
                rects.append((x1, y1, x2, y2))

            objects = ct.update(rects)

            if closest_face_coords:
                x1, y1, x2, y2 = closest_face_coords
                print(f"Closest face detected at x1={x1}, y1={y1}, x2={x2}, y2={y2}")
                print(f"Center of face: x={face_center_x}, y={face_center_y}")
                cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), (0, 0, 255), 2)

                for objectID, bbox in objects.items():
                    if (bbox == (x1, y1, x2, y2)):
                        tracked_object_id = objectID
                        break

                tracking = True
        else:
            objects = ct.update([])  # Pass an empty list to just update the tracker without new detections
            if tracked_object_id in objects:
                bbox = objects[tracked_object_id]
                x1, y1, x2, y2 = bbox
                face_center_x = (x1 + x2) // 2
                face_center_y = (y1 + y2) // 2
                print(f"Tracking face center: x={face_center_x}, y={face_center_y}")
                cv2.rectangle(frame, (x1, y1), (x2, y2), (255, 0, 0), 2)
            else:
                tracking = False

        cv2.imshow('Camera', frame)

        if cv2.waitKey(1) != -1:
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
