from ultralytics import YOLO

model = YOLO("best.pt")

# using image as sourc
results = model.predict(source="image.jpg", show=True, conf=0.35, iou=0.45, save=True)  # includes NMS

# using video as source
# results = model.predict(source="sample-image.mp4", show=True, conf=0.25, iou=0.45, save=True)  # includes NMS

# using webcam as source
# results = model.predict(source=0, show=True, conf=0.25, iou=0.45, save=True)  # includes NMS


