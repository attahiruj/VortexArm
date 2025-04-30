from ultralytics import YOLO


model = YOLO("yolov8mSpice.pt")

results = model.predict(source=1, show=True, conf=0.25, iou=0.45, save=True)  # includes NMS

