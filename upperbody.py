import cv2
import time
# Load the pre-trained Haar cascade classifier for face detection
face_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_upperbody.xml')

# Start video capture from the webcam
cap = cv2.VideoCapture(0)

while True:
    # Read a frame from the video capture
    startTime = time.time()
    ret, frame = cap.read()
    
    # Convert the frame to grayscale (face detection works better on grayscale images)
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    
    # Detect faces in the grayscale image
    faces = face_cascade.detectMultiScale(gray, scaleFactor=1.1, minNeighbors=5, minSize=(30, 30))
    
    # Draw a rectangle around each detected face
    for (x, y, w, h) in faces:
        cv2.rectangle(frame, (x, y), (x + w, y + h), (255, 0, 0), 2)
    
    # Display the resulting frame with rectangles drawn around detected faces
    cv2.imshow('Face Tracker', frame)
    endTime = time.time()
    fps = 1 / (endTime - startTime)
    print(f'FPS: {fps:.2f}')
    print(f'time: {endTime - startTime:.2f}')
    # Break the loop when 'q' key is pressed
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release the video capture object and close all OpenCV windows
cap.release()
cv2.destroyAllWindows()