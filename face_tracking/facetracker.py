import os
import cv2
import datetime
import imutils
import sys
import time
import json
import dlib
import numpy as np
from zerobase import ZeroBase, ZeroBasePubConfig, ZeroBaseSubConfig
from centroidtracker import CentroidTracker

def print_results(performance_stats):
    # Calculate the average processing time per frame
    avg_processing_time = performance_stats['total_time'] / performance_stats['frame_count'] if performance_stats['frame_count'] > 0 else 0

    # Print the performance results
    print("Total processing time: {:.1f} s".format(performance_stats['total_time'] / 1000))
    print("Total frames processed: {}".format(performance_stats['frame_count']))
    print("Average processing time per frame: {:.2f} ms".format(avg_processing_time))

# Main function that is called by zerobase (in an infinite loop)
def main(centroid_tracker, detector, landmark_predictor, video_cap, performance_stats, data, object_no_frames):
    # Read the frame from the video capture
    ret, frame = video_cap.read()

    if frame is None:
        print_results(performance_stats)
        return False  # breaks the loop

    # Resize the frame for faster processing
    frame = imutils.resize(frame, width=600)
    (H, W) = frame.shape[:2]

    # Preprocess the frame for face detection
    face_blob = cv2.dnn.blobFromImage(cv2.resize(frame, (300, 300)), 1.0, (300, 300), (104.0, 177.0, 123.0), False,
                                      False)

    # Set the input to the face detection model
    detector.setInput(face_blob)

    # Perform face detection
    start_time = datetime.datetime.now()  # Start the timer
    face_detections = detector.forward()
    end_time = datetime.datetime.now()  # Stop the timer

    processing_time = (end_time - start_time).total_seconds() * 1000  # Calculate the processing time in milliseconds
    performance_stats['total_time'] += processing_time
    performance_stats['frame_count'] += 1

    rects = []  # List to store detected face bounding boxes

    largest_face = None
    largest_area = 0

    closest_face = None
    closest_distance = float('inf')

    # Iterate over the detected faces
    for i in np.arange(0, face_detections.shape[2]):
        confidence = face_detections[0, 0, i, 2]

        # Filter out weak detections based on confidence threshold
        if confidence > 0.7:
            # Compute the coordinates of the face bounding box
            face_box = face_detections[0, 0, i, 3:7] * np.array([W, H, W, H])
            (startX, startY, endX, endY) = face_box.astype("int")

            face_area = (endX - startX) * (endY - startY)

            if face_area > largest_area:
                largest_area = face_area
                largest_face = face_box

            # Calculate the center of the face bounding box
            face_center_x = (startX + endX) // 2
            face_center_y = (startY + endY) // 2

            # Calculate the distance between the face center and the reference point (e.g., center of the frame)
            reference_point_x = W // 2  # X-coordinate of the reference point
            reference_point_y = H // 2  # Y-coordinate of the reference point
            distance = ((face_center_x - reference_point_x) ** 2 + (face_center_y - reference_point_y) ** 2) ** 0.5

            # Check if the current face is closer than the previous closest face
            if distance < closest_distance:
                closest_distance = distance
                closest_face = face_box
            
            face_info = {
                "frame_number": performance_stats['frame_count'],
                "timestamp": time.time(),
                "image_dimension": [W, H],
                "bounding_box": face_box.tolist(),
            }

            data.append(face_info)

            rects.append(face_box)  # Add the face bounding box to the list of rects
    
    for face_box in rects:
        # Convert the face bounding box to a dlib rectangle object
        dlib_rect = dlib.rectangle(int(face_box[0]), int(face_box[1]), int(face_box[2]), int(face_box[3]))

        # Perform landmark detection
        landmarks = landmark_predictor(frame, dlib_rect)

        # Draw the landmarks on the frame
        for i in range(68):
            x = landmarks.part(i).x
            y = landmarks.part(i).y
            cv2.circle(frame, (x, y), 1, (0, 255, 0), -1)

    if closest_face is not None:
        (startX, startY, endX, endY) = closest_face.astype("int")
        cv2.rectangle(frame, (startX, startY), (endX, endY), (0, 255, 0), 2)
        text = "ID: Closest Face"
        cv2.putText(frame, text, (startX, startY - 5), cv2.FONT_HERSHEY_COMPLEX_SMALL, 1, (0, 255, 0), 1)
        message = json.dumps({"x": int(startX), "y": int(startY)})
        zb.send("eyes/position", message)
        print("Closest Face Coordinates: ({}, {})".format(startX, startY))
    elif largest_face is not None:
        (startX, startY, endX, endY) = largest_face.astype("int")
        cv2.rectangle(frame, (startX, startY), (endX, endY), (0, 0, 255), 2)
        text = "ID: Largest Face"
        cv2.putText(frame, text, (startX, startY - 5), cv2.FONT_HERSHEY_COMPLEX_SMALL, 1, (0, 0, 255), 1)
        print("Largest Face Coordinates: ({}, {})".format(startX, startY))

    # Update the CentroidTracker with the detected face bounding boxes
    objects = centroid_tracker.update(rects)

    # Draw bounding boxes and IDs for tracked faces
    for (objectId, bbox) in objects.items():
        x1, y1, x2, y2 = bbox.astype("int")
        if objectId in object_no_frames:
            object_no_frames[objectId]["frames"] += 1
        else:
            object_no_frames[objectId] = {
                "x1": int(bbox[0]),
                "y1": int(bbox[1]),
                "x2": int(bbox[2]),
                "y2": int(bbox[3]),
                "frames": 1
            }
        cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 0, 255), 2)  # Draw bounding box around the tracked face
        text = "ID: {}".format(objectId)
        cv2.putText(frame, text, (x1, y1 - 5), cv2.FONT_HERSHEY_COMPLEX_SMALL, 1, (0, 0, 255), 1)  # Add ID label
    cv2.imshow("Results", frame)
    key = cv2.waitKey(1)

    # Press 'q' to quit the program
    if key == ord("q"):
        print_results(performance_stats)
        output_filename = "output.json"
        with open(output_filename, "w") as output_file:
            json.dump(object_no_frames, output_file, indent=4, separators=(",", ": "))
        return False  # breaks the loop

    return True  # keeps the loop going

if __name__ == "__main__":

    data = []

    object_no_frames = {}

    # Path to the face detection model files
    protopath = "model/face/deploy.prototxt"
    modelpath = "model/face/res10_300x300_ssd_iter_140000.caffemodel"
    landmarkpath = "./model/landmark_detection/shape_predictor_68_face_landmarks.dat"

    # Check if CUDA argument is provided
    use_cuda = False
    if len(sys.argv) > 1 and sys.argv[1].lower() == "cuda":
        use_cuda = True

    # Load the face detection model
    detector = cv2.dnn.readNetFromCaffe(protopath, modelpath)

    # Load the landmark detection model
    landmark_predictor = dlib.shape_predictor(landmarkpath)

    if use_cuda:
        detector.setPreferableBackend(cv2.dnn.DNN_BACKEND_CUDA)
        detector.setPreferableTarget(cv2.dnn.DNN_TARGET_CUDA)

    # Initialize the CentroidTracker for face tracking
    centroid_tracker = CentroidTracker(90, 90)

    # Open the video capture
    video_cap = cv2.VideoCapture(0)
    # Skip 120 ms of the video
    video_cap.set(cv2.CAP_PROP_POS_MSEC, 2000)

    performance_stats = {'total_time': 0, 'frame_count': 0}  # Map to store performance results

    # Initialize zerobase and its configuration (addresses and topics are very subject to change)
    pub_config = ZeroBasePubConfig("tcp://*:5555")
    sub_config = ZeroBaseSubConfig("tcp://localhost:5556", topics=[""])
    zb = ZeroBase(pub_configs=[pub_config], sub_configs=[sub_config],
                  main=lambda: main(centroid_tracker, detector, landmark_predictor, video_cap, performance_stats, data, object_no_frames))

    # Run zerobase
    zb.run()

    with open("face_info.json", "w") as f:
        json.dump(data, f, indent=4, separators=(",", ": "))

    # Release resources
    video_cap.release()
    cv2.destroyAllWindows()
