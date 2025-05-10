import os
import time
import cv2
import numpy as np
from ultralytics import YOLO
from datetime import datetime

# Original class labels from the model
original_classes = ['BJT', 'LED', 'burnt', 'capacitor', 'cracked', 'faded', 'missing-leg', 'resistor', 'rust']

# Define defect-related classes
defect_classes = {'burnt', 'cracked', 'faded', 'missing-leg', 'rust'}

# Define effective classes to track
count_classes = ['BJT', 'LED', 'capacitor', 'resistor', 'defective']

# Load YOLOv8 Edge TPU TFLite model
model = YOLO('ecomp-detect-yolov8n_edgetpu.tflite', task='detect')

# Initialize video capture
cap = cv2.VideoCapture(0)
if not cap.isOpened():
    print("Failed to open webcam.")
    exit()

print("Live inference with counting started. Press 'q' to quit.")

# Vertical counting line position
line_x = 150
offset = 10
min_distance = 30
centroid_timeout = 1.0

# Initialize counts
counts = {cls: 0 for cls in count_classes}
seen_centroids = {cls: [] for cls in count_classes}

# Helper: calculate centroid
def get_centroid(xyxy):
    x1, y1, x2, y2 = xyxy
    return int((x1 + x2) / 2), int((y1 + y2) / 2)

while True:
    tic = time.time()
    ret, frame = cap.read()
    if not ret:
        break

    result = model(frame, imgsz=256, conf=0.7, verbose=False)
    detections = result[0].boxes
    annotated_frame = result[0].plot()

    current_time = time.time()
    for cls in count_classes:
        seen_centroids[cls] = [(cx, cy, t) for cx, cy, t in seen_centroids[cls]
                               if current_time - t < centroid_timeout]

    if detections is not None and detections.xyxy is not None:
        for i, box in enumerate(detections.xyxy):
            cls_id = int(detections.cls[i].item())
            original_class = original_classes[cls_id]
            class_name = 'defective' if original_class in defect_classes else original_class

            if class_name not in counts:
                continue  # Skip any classes not being tracked

            x1, y1, x2, y2 = map(int, box)
            cx, cy = get_centroid((x1, y1, x2, y2))
            cv2.circle(annotated_frame, (cx, cy), 5, (0, 255, 255), -1)

            if abs(cx - line_x) < offset:
                already_counted = False
                for prev_cx, prev_cy, prev_time in seen_centroids[class_name]:
                    distance = np.sqrt((cx - prev_cx) ** 2 + (cy - prev_cy) ** 2)
                    if distance < min_distance:
                        already_counted = True
                        break

                if not already_counted:
                    counts[class_name] += 1
                    seen_centroids[class_name].append((cx, cy, current_time))
                    print(f"Detected {class_name} at {datetime.now().strftime('%H:%M:%S')} - Count: {counts[class_name]}")

    # Draw counting line
    cv2.line(annotated_frame, (line_x, 0), (line_x, annotated_frame.shape[0]), (255, 0, 0), 2)

    # Show class counts
    y_offset = 30
    for i, (cls, count) in enumerate(counts.items()):
        cv2.putText(annotated_frame, f"{cls}: {count}", (10, y_offset + i * 25),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

    # Show FPS
    fps = 1.0 / (time.time() - tic)
    cv2.putText(annotated_frame, f'FPS: {fps:.2f}', (annotated_frame.shape[1] - 150, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

    cv2.imshow('Edge TPU Detection + Counting', annotated_frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
