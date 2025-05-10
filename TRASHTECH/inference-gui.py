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
import tkinter as tk
from tkinter import ttk, scrolledtext
from PIL import Image, ImageTk

class ComponentDetectionGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("Waste Detection System")
        self.root.attributes('-fullscreen', True)
        
        # Set application state
        self.running = False
        self.shutdown_flag = False
        
        # Create the main frame with 3 columns
        self.main_frame = ttk.Frame(root)
        self.main_frame.pack(fill=tk.BOTH, expand=True)
        
        # Configure the 3 columns with weights
        self.main_frame.columnconfigure(0, weight=1)  # Column 1: Detection count
        self.main_frame.columnconfigure(1, weight=3)  # Column 2: Live capture and log
        self.main_frame.columnconfigure(2, weight=1)  # Column 3: Controls
        self.main_frame.rowconfigure(0, weight=1)     # Main row
        
        # Initialize components and create GUI elements
        self.setup_arduino()
        self.setup_detection_model()
        self.create_column1()  # Detection counts
        self.create_column2()  # Video feed and log
        self.create_column3()  # Control buttons
        
        # Set up video capture variable (will be initialized when starting)
        self.cap = None
        
        # Frame update delay (milliseconds)
        self.update_delay = 33  # ~30 FPS
        
        # Variables for object detection
        self.counts = {cls: 0 for cls in self.count_classes}
        self.seen_centroids = {cls: [] for cls in self.count_classes}
        self.active_objects = defaultdict(list)
        self.last_detection_time = time.time()
        
        # Register window close event
        self.root.protocol("WM_DELETE_WINDOW", self.on_close)
        
        # Log initial message
        self.log("System initialized and ready.")

    def setup_arduino(self):
        # Arduino setup variables
        self.arduino = None
        try:
            self.arduino = serial.Serial('/dev/ttyACM0', 9600, timeout=1)
            time.sleep(2)  # Allow Arduino time to reset
            self.log("Arduino connection established.")
        except Exception as e:
            self.log(f"Arduino connection failed: {e}")
        
        # Queue and cooldown management
        self.component_queue = queue.Queue()
        self.cooldown_tracker = defaultdict(lambda: 0)
        self.cooldown_period = 5  # seconds
        
        # Start Arduino communication thread
        self.arduino_thread = threading.Thread(target=self.arduino_communication, daemon=True)
        self.arduino_thread.start()

    def arduino_communication(self):
        while not self.shutdown_flag:
            try:
                component = self.component_queue.get(timeout=1)
                if component and self.arduino:
                    self.log(f"Sending to Arduino: {component}")
                    self.arduino.write(component.encode())

                    # Wait for Arduino reply
                    response_timeout = time.time() + 2
                    while time.time() < response_timeout and not self.shutdown_flag:
                        if self.arduino.in_waiting > 0:
                            response = self.arduino.readline().decode().strip()
                            self.log(f"Arduino response: {response}")
                            if response == "DONE":
                                break
                    else:
                        self.log("Arduino response timeout.")
            except queue.Empty:
                time.sleep(0.5)
            except Exception as e:
                self.log(f"Arduino communication error: {e}")
                time.sleep(1)

    def setup_detection_model(self):
        # Detection classes
        self.original_classes = ['Biodegradable', 'Non-biodegradable', 'Recyclable']
        self.count_classes = ['Biodegradable', 'Non-biodegradable', 'Recyclable']
        
        # Load YOLO model
        try:
            self.model = YOLO('/home/thesis/Downloads/model_-11-may-2025-0_56_edgetpu.tflite', task='detect')
            self.log("YOLOv8 model loaded successfully.")
        except Exception as e:
            self.log(f"Error loading YOLOv8 model: {e}")
            self.model = None
        
        # Detection parameters
        self.min_distance = 30
        self.centroid_timeout = 5.0
        self.detection_cooldown = 3.0  # 3 seconds delay after detection

    def create_column1(self):
        # Create frame for Column 1 (Detection counts)
        self.count_frame = ttk.LabelFrame(self.main_frame, text="Detection Counts")
        self.count_frame.grid(row=0, column=0, padx=10, pady=10, sticky="nsew")
        
        # Create styles for labels
        style = ttk.Style()
        style.configure('CountLabel.TLabel', font=('Arial', 14))
        style.configure('StatusLabel.TLabel', font=('Arial', 14, 'bold'))
        
        # Create count labels for each class
        self.count_labels = {}
        for i, cls in enumerate(self.count_classes):
            label = ttk.Label(self.count_frame, text=f"{cls}: 0", style='CountLabel.TLabel')
            label.pack(pady=15, padx=10, anchor="w")
            self.count_labels[cls] = label
        
        # Separator
        ttk.Separator(self.count_frame, orient="horizontal").pack(fill="x", pady=10)
        
        # Add FPS display
        self.fps_label = ttk.Label(self.count_frame, text="FPS: 0.0", style='CountLabel.TLabel')
        self.fps_label.pack(pady=15, padx=10, anchor="w")
        
        # Add status display
        self.status_label = ttk.Label(self.count_frame, text="Status: Stopped", style='StatusLabel.TLabel')
        self.status_label.pack(pady=15, padx=10, anchor="w")

    def create_column2(self):
        # Create frame for Column 2 (Video + Log)
        self.col2_frame = ttk.Frame(self.main_frame)
        self.col2_frame.grid(row=0, column=1, padx=10, pady=10, sticky="nsew")
        
        # Configure rows for video (70%) and log (30%)
        self.col2_frame.rowconfigure(0, weight=7)  # Video gets 70%
        self.col2_frame.rowconfigure(1, weight=3)  # Log gets 30%
        self.col2_frame.columnconfigure(0, weight=1)
        
        # Create video frame
        self.video_frame = ttk.LabelFrame(self.col2_frame, text="Live Detection")
        self.video_frame.grid(row=0, column=0, padx=5, pady=5, sticky="nsew")
        
        # Create video label
        self.video_label = ttk.Label(self.video_frame)
        self.video_label.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)
        
        # Create log frame
        self.log_frame = ttk.LabelFrame(self.col2_frame, text="System Log")
        self.log_frame.grid(row=1, column=0, padx=5, pady=5, sticky="nsew")
        
        # Create scrolled text for log
        self.log_text = scrolledtext.ScrolledText(self.log_frame, wrap=tk.WORD)
        self.log_text.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)

    def create_column3(self):
        # Create frame for Column 3 (Controls)
        self.control_frame = ttk.LabelFrame(self.main_frame, text="Controls")
        self.control_frame.grid(row=0, column=2, padx=10, pady=10, sticky="nsew")
        
        # Create a style for buttons
        style = ttk.Style()
        style.configure('Large.TButton', font=('Arial', 14))
        
        # Create buttons with consistent size
        self.start_button = ttk.Button(self.control_frame, text="Start Detection", 
                                      command=self.start_detection, width=20, style='Large.TButton')
        self.start_button.pack(pady=20, padx=10)
        
        self.stop_button = ttk.Button(self.control_frame, text="Stop Detection", 
                                     command=self.stop_detection, width=20, style='Large.TButton')
        self.stop_button.pack(pady=20, padx=10)
        self.stop_button.config(state=tk.DISABLED)
        
        self.shutdown_button = ttk.Button(self.control_frame, text="Shutdown", 
                                         command=self.shutdown, width=20, style='Large.TButton')
        self.shutdown_button.pack(pady=20, padx=10)
        
        # Add additional system info
        ttk.Separator(self.control_frame, orient="horizontal").pack(fill="x", pady=30)
        
        # Create style for info labels
        style = ttk.Style()
        style.configure('Info.TLabel', font=('Arial', 12))
        
        # Time info
        self.time_label = ttk.Label(self.control_frame, text="", style='Info.TLabel')
        self.time_label.pack(pady=10, padx=10)
        self.update_time()
        
        # Last detection info
        self.last_detection_label = ttk.Label(self.control_frame, 
                                             text="Time since last detection: 0.0s", 
                                             style='Info.TLabel')
        self.last_detection_label.pack(pady=10, padx=10)

    def update_time(self):
        """Update the time display"""
        current_time = datetime.now().strftime("%H:%M:%S")
        current_date = datetime.now().strftime("%Y-%m-%d")
        self.time_label.config(text=f"Date: {current_date}\nTime: {current_time}")
        self.root.after(1000, self.update_time)

    def log(self, message):
        """Add message to log with timestamp"""
        timestamp = datetime.now().strftime("%H:%M:%S")
        log_entry = f"[{timestamp}] {message}\n"
        
        # Use after to ensure thread safety when updating GUI
        self.root.after(0, lambda: self.log_text.insert(tk.END, log_entry))
        self.root.after(0, lambda: self.log_text.see(tk.END))

    def start_detection(self):
        """Start the detection process"""
        if self.running:
            return
            
        self.log("Starting detection...")
        
        try:
            # Initialize camera
            self.cap = cv2.VideoCapture(0)
            if not self.cap.isOpened():
                self.log("ERROR: Failed to open webcam.")
                return
                
            # Get frame dimensions
            self.frame_w = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
            self.frame_h = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
            
            # Reset detection counters
            self.counts = {cls: 0 for cls in self.count_classes}
            self.update_count_display()
            
            # Set flags
            self.running = True
            self.status_label.config(text="Status: Running")
            
            # Update button states
            self.start_button.config(state=tk.DISABLED)
            self.stop_button.config(state=tk.NORMAL)
            
            # Start detection loop
            self.update_frame()
            
        except Exception as e:
            self.log(f"Error starting detection: {e}")
            self.running = False
            self.status_label.config(text="Status: Error")

    def stop_detection(self):
        """Stop the detection process"""
        self.running = False
        self.status_label.config(text="Status: Stopped")
        
        # Update button states
        self.start_button.config(state=tk.NORMAL)
        self.stop_button.config(state=tk.DISABLED)
        
        # Release camera
        if self.cap:
            self.cap.release()
            self.cap = None
            
        self.log("Detection stopped.")

    def shutdown(self):
        """Shutdown the application"""
        self.log("Shutting down...")
        self.shutdown_flag = True
        self.stop_detection()
        time.sleep(1)  # Give threads time to clean up
        self.root.destroy()

    def on_close(self):
        """Handle window close event"""
        self.shutdown()

    def get_centroid(self, xyxy):
        """Calculate centroid of a bounding box"""
        x1, y1, x2, y2 = xyxy
        return int((x1 + x2) / 2), int((y1 + y2) / 2)

    def update_count_display(self):
        """Update the count labels"""
        for cls, count in self.counts.items():
            self.count_labels[cls].config(text=f"{cls}: {count}")

    def update_frame(self):
        """Process a frame and update the GUI"""
        if not self.running or not self.cap:
            return
            
        # Measure processing time for FPS calculation
        tic = time.time()
        
        # Read frame
        ret, frame = self.cap.read()
        if not ret:
            self.log("Error: Could not read frame from camera")
            self.stop_detection()
            return
            
        # Process frame with YOLO
        if self.model:
            result = self.model(frame, imgsz=256, conf=0.7, verbose=False)
            detections = result[0].boxes
            annotated_frame = result[0].plot()
        else:
            # If model failed to load, just use the original frame
            annotated_frame = frame.copy()
            detections = None
            
        current_time = time.time()
        
        # Clean up outdated centroids
        for cls in self.count_classes:
            self.seen_centroids[cls] = [(cx, cy, t) for cx, cy, t in self.seen_centroids[cls] 
                                       if current_time - t < self.centroid_timeout]

        # Cleanup active_objects after timeout
        for cls in list(self.active_objects.keys()):
            self.active_objects[cls] = [(cx, cy, t) for cx, cy, t in self.active_objects[cls] 
                                       if current_time - t < self.centroid_timeout]

        # Process detections
        if detections is not None and detections.xyxy is not None and len(detections.xyxy) > 0:
            for i, box in enumerate(detections.xyxy):
                cls_id = int(detections.cls[i].item())
                class_name = self.original_classes[cls_id]

                if class_name not in self.counts:
                    continue

                x1, y1, x2, y2 = map(int, box)
                cx, cy = self.get_centroid((x1, y1, x2, y2))
                
                # Draw bounding box around detected object
                cv2.rectangle(annotated_frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.circle(annotated_frame, (cx, cy), 5, (0, 255, 255), -1)
                
                self.last_detection_time = current_time
                
                already_tracked = False
                for prev_cx, prev_cy, _ in self.active_objects[class_name]:
                    distance = np.sqrt((cx - prev_cx) ** 2 + (cy - prev_cy) ** 2)
                    if distance < self.min_distance:
                        already_tracked = True
                        break

                if not already_tracked:
                    self.counts[class_name] += 1
                    self.update_count_display()
                    self.seen_centroids[class_name].append((cx, cy, current_time))
                    self.active_objects[class_name].append((cx, cy, current_time))
                    self.log(f"Detected {class_name}")

                    if current_time - self.cooldown_tracker[class_name] > self.cooldown_period:
                        self.cooldown_tracker[class_name] = current_time
                        if class_name == 'Biodegradable':
                            self.component_queue.put('BIO\n')
                        elif class_name == 'Non-biodegradable':
                            self.component_queue.put('NONBIO\n')
                        elif class_name == 'Recyclable':
                            self.component_queue.put('RECY\n')
                        
                        # Add 3-second delay after detection
                        time.sleep(3)
        
        # Only process detections if cooldown period has passed
        current_time = time.time()
        if current_time - self.last_detection_time < self.detection_cooldown:
            cv2.putText(annotated_frame, 'Detection Paused...', (10, self.frame_h - 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

        # Calculate and display FPS
        fps = 1.0 / (time.time() - tic)
        self.fps_label.config(text=f"FPS: {fps:.2f}")
        cv2.putText(annotated_frame, f'FPS: {fps:.2f}', (annotated_frame.shape[1] - 150, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

        # Convert to PIL format and display in Tkinter
        cv_image = cv2.cvtColor(annotated_frame, cv2.COLOR_BGR2RGB)
        pil_image = Image.fromarray(cv_image)
        
        # Resize to fit in the frame if needed
        video_width = self.video_label.winfo_width()
        video_height = self.video_label.winfo_height()
        
        if video_width > 1 and video_height > 1:  # Ensure frame is initialized
            pil_image = pil_image.resize((video_width, video_height), Image.LANCZOS)
            
        # Convert to Tkinter format and update the label
        tk_image = ImageTk.PhotoImage(image=pil_image)
        self.video_label.imgtk = tk_image  # Keep a reference to prevent garbage collection
        self.video_label.config(image=tk_image)
        
        # Schedule the next frame update
        if self.running:
            self.root.after(self.update_delay, self.update_frame)

# Main application entry point
if __name__ == "__main__":
    root = tk.Tk()
    app = ComponentDetectionGUI(root)
    root.mainloop()