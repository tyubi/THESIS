from ultralytics import YOLO
import os
import cv2
import serial  # For serial communication
import time
import tkinter as tk
from tkinter import ttk
from PIL import Image, ImageTk
import threading

# Initialize serial communication with Arduino
arduino = serial.Serial('COM6', 9600, timeout=1)  # Replace 'COM6' with your Arduino's port
time.sleep(2)  # Allow time for the connection to establish

# Load the YOLOv8 model
model = YOLO("best.pt")

# Function to initialize the camera
def initialize_camera():
    global cap
    cap = cv2.VideoCapture(0)  # Use DirectShow backend for faster initialization

    if not cap.isOpened():
        print("Error: Could not open webcam.")
        return False

    # Warm up the camera by discarding initial frames
    for _ in range(10):
        cap.read()

    print("Camera initialized successfully.")
    return True

# Function to handle YOLO detection and video feed
def start_detection():
    if not initialize_camera():
        status_label.config(text="Error: Could not initialize camera.")
        return

    while True:
        ret, frame = cap.read()
        if not ret:
            print("Error: Failed to capture frame.")
            break

        # Run YOLO inference on the frame
        results = model(frame)

        # Process detection results
        for detection in results[0].boxes:
            cls = int(detection.cls)  # Get the class index
            confidence = detection.conf  # Get confidence score

            if confidence > 0.70:  # Process only high-confidence detections
                if cls == 0:  # BJT detected
                    arduino.write(b'A')
                    status_label.config(text="Status: BJT detected!")
                elif cls == 1:  # LED detected
                    arduino.write(b'B')
                    status_label.config(text="Status: LED detected!")
                elif cls == 2:  # Capacitor detected
                    arduino.write(b'C')
                    status_label.config(text="Status: Capacitor detected!")
                elif cls == 3:  # Defective component detected
                    arduino.write(b'D')
                    status_label.config(text="Status: Defective component detected!")
                elif cls == 4:  # Resistor detected
                    arduino.write(b'E')
                    status_label.config(text="Status: Resistor detected!")
                else:
                    arduino.write(b'F')  # Unknown
                    status_label.config(text="Status: Unknown component detected!")
            else:  # Low-confidence detections
                arduino.write(b'F')  # Unknown
                status_label.config(text="Status: Unknown component detected!")

        # Visualize the results on the frame
        annotated_frame = results[0].plot()

        # Convert frame to RGB and display in the GUI
        rgb_frame = cv2.cvtColor(annotated_frame, cv2.COLOR_BGR2RGB)
        img = Image.fromarray(rgb_frame)
        imgtk = ImageTk.PhotoImage(image=img)
        video_label.imgtk = imgtk
        video_label.configure(image=imgtk)

        if stop_event.is_set():
            break

    cap.release()
    arduino.close()

# Function to start detection in a separate thread
def run_detection():
    global stop_event
    stop_event.clear()
    threading.Thread(target=start_detection).start()

# Function to stop detection
def stop_detection():
    global stop_event
    stop_event.set()
    status_label.config(text="Status: Detection stopped.")

# Initialize the GUI
root = tk.Tk()
root.title("Electronic Component Detection")

# GUI Widgets
video_label = tk.Label(root)
video_label.pack()

status_label = tk.Label(root, text="Status: Waiting to start...", font=("Arial", 14))
status_label.pack(pady=10)

control_frame = ttk.Frame(root)
control_frame.pack()

start_button = ttk.Button(control_frame, text="Start Detection", command=run_detection)
start_button.grid(row=0, column=0, padx=10, pady=10)

stop_button = ttk.Button(control_frame, text="Stop Detection", command=stop_detection)
stop_button.grid(row=0, column=1, padx=10, pady=10)

quit_button = ttk.Button(control_frame, text="Quit", command=root.destroy)
quit_button.grid(row=0, column=2, padx=10, pady=10)

# Stop event for stopping threads
stop_event = threading.Event()

# Run the GUI
root.mainloop()
