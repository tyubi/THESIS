import os
import time
import cv2
import numpy as np
from ultralytics import YOLO
from datetime import datetime
import serial
import threading
import queue
from collections import defaultdict

# -------------------------------------
# Arduino Setup
# -------------------------------------
arduino = serial.Serial('/dev/ttyACM0', 9600, timeout=1)  # Change COM port if needed
time.sleep(2)  # Allow Arduino time to reset

# Queue and cooldown management
component_queue = queue.Queue()
cooldown_tracker = defaultdict(lambda: 0)
cooldown_period = 5  # seconds

# Active object tracker
active_objects = defaultdict(list)  # class -> list of (cx, cy)

# Arduino communication thread
def arduino_communication():
    while True:
        try:
            component = component_queue.get(timeout=1)
            if component:
                print(f"Sending to Arduino: {component}")
                arduino.write(component.encode())

                # Wait for Arduino reply
                response_timeout = time.time() + 2
                while time.time() < response_timeout:
                    if arduino.in_waiting > 0:
                        response = arduino.readline().decode().strip()
                        print(f"Arduino response: {response}")
                        if response == "DONE":
                            break
                else:
                    print("Arduino response timeout.")
        except queue.Empty:
            time.sleep(0.5)

# Start Arduino communication
arduino_thread = threading.Thread(target=arduino_communication, daemon=True)
arduino_thread.start()

# -------------------------------------
# YOLOv8 Detection + Counting + Arduino
# -------------------------------------
original_classes = ['Biodegradable', 'Non-biodegradable', 'Recyclable']
count_classes = ['Biodegradable', 'Non-biodegradable', 'Recyclable']

model = YOLO('/home/thesis/Downloads/model_-11-may-2025-0_56_edgetpu.tflite', task='detect')

cap = cv2.VideoCapture(0)
if not cap.isOpened():
    print("Failed to open webcam.")
    exit()

# Set fixed frame dimensions
frame_w = 440
frame_h = 440
cap.set(cv2.CAP_PROP_FRAME_WIDTH, frame_w)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, frame_h)

# Verify that the settings were applied
actual_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
actual_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
print(f"Frame dimensions set to: Width = {actual_w}, Height = {actual_h}")

print("Live inference with counting started. Press 'q' to quit.")

min_distance = 30
centroid_timeout = 5.0

# Add delay tracking
last_detection_time = time.time()
detection_cooldown = 3.0  # 3 seconds delay after detection

counts = {cls: 0 for cls in count_classes}
seen_centroids = {cls: [] for cls in count_classes}

def get_centroid(xyxy):
    x1, y1, x2, y2 = xyxy
    return int((x1 + x2) / 2), int((y1 + y2) / 2)

while True:
    tic = time.time()
    ret, frame = cap.read()
    if not ret:
        break

    # Update model inference with correct image size (256x256)
    result = model(frame, imgsz=256, conf=0.7, verbose=False)
    detections = result[0].boxes
    annotated_frame = result[0].plot()

    current_time = time.time()
    
    # Check for any detections
    found_detection = False

    for cls in count_classes:
        seen_centroids[cls] = [(cx, cy, t) for cx, cy, t in seen_centroids[cls] if current_time - t < centroid_timeout]

    # Cleanup active_objects after timeout
    for cls in list(active_objects.keys()):
        active_objects[cls] = [(cx, cy, t) for cx, cy, t in active_objects[cls] if current_time - t < centroid_timeout]

    if detections is not None and detections.xyxy is not None and len(detections.xyxy) > 0:
        for i, box in enumerate(detections.xyxy):
            cls_id = int(detections.cls[i].item())
            class_name = original_classes[cls_id]

            if class_name not in counts:
                continue

            x1, y1, x2, y2 = map(int, box)
            cx, cy = get_centroid((x1, y1, x2, y2))
            
            # Draw bounding box around detected object
            cv2.rectangle(annotated_frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.circle(annotated_frame, (cx, cy), 5, (0, 255, 255), -1)
            
            found_detection = True
            last_detection_time = current_time
            
            already_tracked = False
            for prev_cx, prev_cy, _ in active_objects[class_name]:
                distance = np.sqrt((cx - prev_cx) ** 2 + (cy - prev_cy) ** 2)
                if distance < min_distance:
                    already_tracked = True
                    break

            if not already_tracked:
                counts[class_name] += 1
                seen_centroids[class_name].append((cx, cy, current_time))
                active_objects[class_name].append((cx, cy, current_time))
                print(f"Detected {class_name} at {datetime.now().strftime('%H:%M:%S')} - Count: {counts[class_name]}")

                if current_time - cooldown_tracker[class_name] > cooldown_period:
                    cooldown_tracker[class_name] = current_time
                    if class_name == 'Biodegradable':
                        component_queue.put('BIO\n')
                    elif class_name == 'Non-biodegradable':
                        component_queue.put('NONBIO\n')
                    elif class_name == 'Recyclable':
                        component_queue.put('RECY\n')
                    
                    # Add 3-second delay after detection
                    time.sleep(3)
    
    # Only process detections if cooldown period has passed
    current_time = time.time()
    if current_time - last_detection_time < detection_cooldown:
        cv2.putText(annotated_frame, 'Detection Paused...', (10, frame_h - 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

    # Display counts
    y_offset = 30
    for i, (cls, count) in enumerate(counts.items()):
        cv2.putText(annotated_frame, f"{cls}: {count}", (10, y_offset + i * 25),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

    # Display FPS
    fps = 1.0 / (time.time() - tic)
    cv2.putText(annotated_frame, f'FPS: {fps:.2f}', (annotated_frame.shape[1] - 150, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

    cv2.imshow('Edge TPU Detection + Counting', annotated_frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()