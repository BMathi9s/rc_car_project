import cv2
import numpy as np
import time

def safe_cascade_load():
    try:
        cascade = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_frontalface_default.xml')
        if cascade.empty():
            raise Exception("Failed to load cascade classifier")
        return cascade
    except Exception as e:
        print(f"Error loading cascade classifier: {e}")
        return None

def safe_camera_open(camera_index=0, max_attempts=5):
    for attempt in range(max_attempts):
        try:
            cap = cv2.VideoCapture(camera_index)
            if not cap.isOpened():
                raise Exception("Failed to open camera")
            return cap
        except Exception as e:
            print(f"Attempt {attempt + 1}/{max_attempts}: Failed to open camera. Error: {e}")
            time.sleep(1)
    print("Failed to open camera after multiple attempts")
    return None

def distance_to_center(face, frame_center):
    x, y, w, h = face
    face_center = (x + w // 2, y + h // 2)
    return np.sqrt((face_center[0] - frame_center[0])**2 + (face_center[1] - frame_center[1])**2)

def main():
    face_cascade = safe_cascade_load()
    if face_cascade is None:
        return

    cap = safe_camera_open()
    if cap is None:
        return

    # Set lower resolution
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)

    face_tracker = None
    track_window = None
    current_target = None
    last_detection_time = time.time()
    frame_count = 0

    while True:
        try:
            ret, frame = cap.read()
            if not ret:
                print("Failed to grab frame, trying to reinitialize camera...")
                cap.release()
                cap = safe_camera_open()
                if cap is None:
                    break
                continue

            frame_count += 1
            if frame_count % 2 != 0:  # Process every other frame
                continue

            frame_center = (frame.shape[1] // 2, frame.shape[0] // 2)
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            
            # Detect faces
            faces = face_cascade.detectMultiScale(gray, scaleFactor=1.05, minNeighbors=3, minSize=(15, 15)) # Can change to scaleFactor=1.15 if performance is too slow. Can even lower to minNeighbors=1 if not many false positives are occuring.

            if len(faces) > 0:
                last_detection_time = time.time()
                # Sort faces by distance to center
                faces = sorted(faces, key=lambda face: distance_to_center(face, frame_center))

                if current_target is None or current_target >= len(faces):
                    current_target = 0

                # Get the current target face
                (x, y, w, h) = faces[current_target]
                
                # Initialize or update the tracker
                if face_tracker is None:
                    track_window = (x, y, w, h)
                    face_tracker = cv2.TrackerKCF_create()
                    face_tracker.init(frame, track_window)
                else:
                    success, track_window = face_tracker.update(frame)
                    if not success:
                        face_tracker = None
                        continue

                x, y, w, h = track_window
                center_x = x + w // 2
                center_y = y + h // 2

                print(f"Face center coordinates: X={center_x}, Y={center_y}")

                # Draw rectangle around the face (comment out for deployment)
                cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 255, 0), 2)
                cv2.circle(frame, (center_x, center_y), 3, (0, 0, 255), -1)
            else:
                # Reset tracker if no face is detected for more than 5 seconds
                if time.time() - last_detection_time > 5:
                    face_tracker = None
                    current_target = None
                    print("No face detected for 5 seconds, resetting tracker")

            # Display the frame (comment out for deployment)
            cv2.imshow('Face Tracking', frame)

            # Handle key presses (adjust for actual input method on the RC car)
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break
            elif key == ord('n'):
                # Switch to the next face
                if faces is not None and len(faces) > 0:
                    current_target = (current_target + 1) % len(faces)
                    face_tracker = None  # Reset tracker to reinitialize on new face

        except Exception as e:
            print(f"An error occurred: {e}")
            time.sleep(1)

    if cap is not None:
        cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()