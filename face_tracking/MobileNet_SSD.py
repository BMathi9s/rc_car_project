import tensorflow as tf
import numpy as np
import cv2

# Load the TFLite model and allocate tensors.
interpreter = tf.lite.Interpreter(model_path=r"MobileNet_V1_SSD_Model\coco_ssd_mobilenet_v1_1.0_quant_2018_06_29\detect.tflite")
interpreter.allocate_tensors()

# Get input and output tensors.
input_details = interpreter.get_input_details()
output_details = interpreter.get_output_details()

with open("MobileNet_V1_SSD_Model\coco_ssd_mobilenet_v1_1.0_quant_2018_06_29\labelmap.txt", "r") as f:
    labels = [line.strip() for line in f.readlines()]

cap = cv2.VideoCapture(0)

# cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
# cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)
cap.set(cv2.CAP_PROP_FPS, 15)  # Set lower FPS for less CPU load

if not cap.isOpened():
    print("Error: Could not open video.")
    exit()

while True:
    ret, frame = cap.read()
    if not ret:
        break

    input_data = cv2.resize(frame, (300, 300))
    input_data = np.expand_dims(input_data, axis=0)
    input_data = input_data.astype(np.uint8)

    # Set the tensor to point to the input data to be inferred.
    interpreter.set_tensor(input_details[0]['index'], input_data)

    # Run the inference.
    interpreter.invoke()

    boxes = interpreter.get_tensor(output_details[0]['index'])[0]  # Bounding box coordinates of detected objects.
    classes = interpreter.get_tensor(output_details[1]['index'])[0]  # Class index of detected objects.
    scores = interpreter.get_tensor(output_details[2]['index'])[0]  # Confidence scores of detected objects.

    # Process and display the results.
    height, width, _ = frame.shape
    for i in range(len(scores)):
        if scores[i] > 0.6:  # Confidence threshold.
            ymin, xmin, ymax, xmax = boxes[i]
            xmin = int(xmin * width)
            xmax = int(xmax * width)
            ymin = int(ymin * height)
            ymax = int(ymax * height)

            cv2.rectangle(frame, (xmin, ymin), (xmax, ymax), (0, 255, 0), 2)
            label = f"{labels[int(classes[i]) + 1]}, {scores[i]:.2f}"
            cv2.putText(frame, label, (xmin, ymin - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

            if int(classes[i] + 1) == 1:
                cv2.putText(frame, "ENEMY SPOTTED", (xmin, ymin - 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

    cv2.imshow('Object Detection', frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
