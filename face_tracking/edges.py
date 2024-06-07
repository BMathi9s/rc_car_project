import cv2
import numpy as np
import matplotlib.pyplot as plt


import cv2

def main():
    # Open the default camera (usually the first one)
    cap = cv2.VideoCapture(0)
    
    # Check if the camera opened successfully
    if not cap.isOpened():
        print("Error: Could not open camera.")
        return

    while True:
        # Capture frame-by-frame
        ret, frame = cap.read()
        
          # Step 2: Convert to grayscale
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        #    Step 3: Apply Gaussian Blur
        blurred = cv2.GaussianBlur(gray, (1, 1), 0)
        # Step 4: Apply Canny Edge Detection
        edges = cv2.Canny(blurred, threshold1=10, threshold2=100)
        # Step 5: Display the original image and the edges
        plt.figure(figsize=(20, 10))
        plt.subplot(1, 3, 1)
        plt.title('Original Image')
        plt.imshow(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
        plt.axis('off')

        plt.subplot(1, 3, 2)
        plt.title('Edges')
        plt.imshow(edges, cmap='gray')
        plt.axis('off')
        plt.subplot(1, 3, 3)
        plt.title('blurr')
        plt.imshow(cv2.cvtColor(blurred, cv2.COLOR_BGR2RGB))
        plt.axis('off')
        

        plt.show()

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

