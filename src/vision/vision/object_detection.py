import numpy as np
import pyarrow as pa

from dora import DoraStatus
from ultralytics import YOLO


CAMERA_WIDTH = 640
CAMERA_HEIGHT = 480


# model = YOLO("yolov8n.pt")
model = YOLO("yolov8mSpice.pt")


class Operator:
    """
    Inferring object from images
    """

    def on_event(
        self,
        dora_event,
        send_output,
    ) -> DoraStatus:
        if dora_event["type"] == "INPUT":
            frame = (
                dora_event["value"].to_numpy().reshape((CAMERA_HEIGHT, CAMERA_WIDTH, 3))
            )
            # frame = frame[:, :, ::-1]  # OpenCV image (BGR to RGB)
            
            # frame = dora_event["value"]
            print(frame.shape)
            print(frame)
            results = model(frame, show=True, conf=0.25, iou=0.45)  # includes NMS
            
            for result in results:
                boxes = result.boxes.cpu().numpy()
                
                xyxy = boxes.xyxy
                print(xyxy)

            # results = model(frame, verbose=False)  # includes NMS
            
            print(results[0])
            # Process results
            boxes = np.array(results[0].boxes.xyxy.cpu())
            conf = np.array(results[0].boxes.conf.cpu())
            label = np.array(results[0].boxes.cls.cpu())
            # concatenate them together
            arrays = np.concatenate((boxes, conf[:, None], label[:, None]), axis=1)

            send_output("bbox", pa.array(arrays.ravel()), dora_event["metadata"])

        return DoraStatus.CONTINUE
