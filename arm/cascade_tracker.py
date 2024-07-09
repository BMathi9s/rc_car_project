import cv2
import os
import time
from turret import Turret

def main():
    haar_cascade_path = cv2.data.haarcascades + 'haarcascade_frontalface_default.xml'
    
    if not os.path.exists(haar_cascade_path):
        print(f"Haar cascade file not found at {haar_cascade_path}.")
        return

    face_cascade = cv2.CascadeClassifier(haar_cascade_path)
    
    cap = cv2.VideoCapture(0)
    
    if not cap.isOpened():
        print("Error: Could not open camera.")
        return
    
    # Initialize the turret
    turret = Turret(base_channel=0, canon_channel=1)
    turret.set_proportionnal_constant(constant=0.02)  # Set the proportional constant

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        faces = face_cascade.detectMultiScale(gray_frame, scaleFactor=1.1, minNeighbors=5, minSize=(30, 30))
        
        max_area = 0
        closest_face_coords = None
        
        for (x, y, w, h) in faces:
            area = w * h
            
            if area > max_area:
                max_area = area
                closest_face_coords = (x, y, w, h)
                face_center_x = x + w // 2
                face_center_y = y + h // 2
                
        if closest_face_coords:
            x, y, w, h = closest_face_coords
            print(f"Closest face detected at x={x}, y={y}, w={w}, h={h}")
            print(f"Center of face: x={face_center_x}, y={face_center_y}")
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 0, 255), 2)
            
            # Calculate the difference from the center of the screen
            screen_center_x = frame.shape[1] // 2
            screen_center_y = frame.shape[0] // 2
            face_diff_x = face_center_x - screen_center_x
            face_diff_y = face_center_y - screen_center_y
            print(f"Face center differences : x={face_diff_x}, y={face_diff_y}")
            
            if abs(face_diff_x) > 20 or abs(face_diff_y) > 20:
                # Update turret position
                turret.update(face_diff_x, face_diff_y)

        # cv2.imshow('Camera', frame)

        # if cv2.waitKey(1) != -1:
        #     break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()