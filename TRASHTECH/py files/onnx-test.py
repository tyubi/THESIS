import onnx
import numpy as np
import cv2
import torch
from onnxruntime import InferenceSession
from PIL import Image

# Load the ONNX model
model_path = "best.onnx"
session = InferenceSession(model_path)

# Function to preprocess the input image
def preprocess_image(image_path):
    image = Image.open(image_path)
    image = image.convert("RGB")
    image = image.resize((640, 640))  # Resize to match input size of YOLOv8 model
    image = np.array(image) / 255.0  # Normalize the image
    image = image.transpose((2, 0, 1))  # Change to CxHxW format
    image = np.expand_dims(image, axis=0).astype(np.float32)  # Add batch dimension
    return image

# Function to run detection
def start_detection():
    image_path = "Electronics Component Detection.v2i.yolov8/test/images/IMG_0344_mov-0012_jpg.rf.608614f1e75e3035993e340a944c8364.jpg"
    image = preprocess_image(image_path)

    # Run inference
    inputs = {session.get_inputs()[0].name: image}
    detections = session.run(None, inputs)[0]  # Get detections

    print("First detection object:", detections[0])  # Print the first detection

    # Post-process detections
    for detection in detections:
        print("Detection:", detection)
        
        # Check if the confidence is an array and extract the value
        x1, y1, x2, y2, confidence, cls = detection
        confidence_value = confidence[0] if isinstance(confidence, np.ndarray) else confidence
        confidence = float(confidence_value)  # Ensure confidence is a float
        
        print(f"Confidence: {confidence}")
        print(f"Class: {cls}")

# Run the detection
start_detection()
