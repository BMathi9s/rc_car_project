# load libraries
import cv2

from huggingface_hub import hf_hub_download
from ultralytics import YOLO
from supervision import Detections
from PIL import Image



def main():
        # download model
    model_path = hf_hub_download(repo_id="arnabdhar/YOLOv8-Face-Detection", filename="model.pt")
    # load model
    model = YOLO(model_path)
    # Open the default camera (usually the first one)
    cap = cv2.VideoCapture(0)
    # Check if the camera opened successfully
    if not cap.isOpened():
        print("Error: Could not open camera.")
        return
    while True:
        # Capture frame-by-frame
        ret, frame = cap.read()

        # Convert the image from BGR color (which OpenCV uses) to RGB color
        rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

        # Perform inference on the RGB image
        output = model(Image.fromarray(rgb_frame))
        results = Detections.from_ultralytics(output[0])
        
        # print(f"RESULTS :  {results} -- {type(results)}")
        
        
        # Print the bounding box coordinates of each detected faceq
        for result in results.xyxy:
            x, y, w, h = result  # If result is a tuple of four elements
            print(f"Face detected at x={x}, y={y}, width={w}, height={h}")
            # Draw rectangle around the face
        #     cv2.rectangle(frame, (int(x), int(y)), (int(w), int(h)), (0, 0, 255), 2)
        # # Display the resulting frame
        # cv2.imshow('Camera', frame)

        # # Break the loop if 'q' is pressed
        # if cv2.waitKey(1) & 0xFF == ord('q'):
        #     break

    # Release the camera and close the window
    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
