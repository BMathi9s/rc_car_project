import cv2

# Initialize the HOG descriptor/person detector
hog = cv2.HOGDescriptor()
hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())

# Initialize the webcam
cap = cv2.VideoCapture(0)

# Set the resolution (optional)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

while cap.isOpened():
    ret, frame = cap.read()
    if not ret:
        break

    # Resize the frame to improve detection speed and accuracy
    frame_resized = cv2.resize(frame, (640, 480))
    
    # Convert the frame to grayscale
    gray = cv2.cvtColor(frame_resized, cv2.COLOR_BGR2GRAY)

    # Detect people in the frame
    boxes, weights = hog.detectMultiScale(gray, winStride=(8, 8), padding=(8, 8), scale=1.05)

    # Filter out the lower half of the detections to focus on upper bodies
    upper_bodies = []
    for (x, y, w, h) in boxes:
        upper_body = (x, y, w, int(h / 2))
        upper_bodies.append(upper_body)

    # Draw rectangles around detected upper bodies
    for (x, y, w, h) in upper_bodies:
        cv2.rectangle(frame_resized, (x, y), (x + w, y + h), (255, 0, 0), 2)

    # Display the resulting frame
    cv2.imshow('Upper Body Detection', frame_resized)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release the webcam and close the window
cap.release()
cv2.destroyAllWindows()