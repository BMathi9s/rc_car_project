import cv2
import numpy as np
import tflite_runtime.interpreter as tflite

# Load TFLite model and allocate tensors
interpreter = tflite.Interpreter(model_path="midas_model.tflite")
interpreter.allocate_tensors()

# Get input and output tensors
input_details = interpreter.get_input_details()
output_details = interpreter.get_output_details()

# Initialize camera
cap = cv2.VideoCapture(0)
if not cap.isOpened():
    raise IOError("Cannot open webcam")

while True:
    ret, frame = cap.read()
    if not ret:
        break

    # Preprocess the frame
    input_frame = cv2.resize(frame, (256, 256))
    input_frame = np.expand_dims(input_frame, axis=0)
    input_frame = np.float32(input_frame) / 255.0

    # Run inference
    interpreter.set_tensor(input_details[0]['index'], input_frame)
    interpreter.invoke()

    # Get the result
    depth_map = interpreter.get_tensor(output_details[0]['index'])[0]

    # Normalize the depth map for visualization
    depth_map = cv2.normalize(depth_map, None, 0, 255, cv2.NORM_MINMAX)
    depth_map = np.uint8(depth_map)
    depth_map = cv2.applyColorMap(depth_map, cv2.COLORMAP_MAGMA)

    # Display the result
    combined_frame = np.hstack((frame, depth_map))
    cv2.imshow('Depth Map and RGB Frame', combined_frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()