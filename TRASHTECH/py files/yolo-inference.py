from ultralytics import YOLO

model = YOLO('YOLOv8s.pt')

results = model(source=0, conf=0.70, show=True, save=True)