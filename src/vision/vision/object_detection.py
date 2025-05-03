import numpy as np
import pyarrow as pa

from dora import DoraStatus
from ultralytics import YOLO


CAMERA_WIDTH = 640
CAMERA_HEIGHT = 480


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
            # print(frame.shape)
            # print(frame)
            results = model(frame, conf=0.45, iou=0.45)  # includes NMS
            # results = model(frame, conf=0.25, iou=0.45)  # includes NMS
            frame_third = CAMERA_WIDTH // 3  # Divide the frame width into 3 parts
            
            for result in results:
                boxes = result.boxes.cpu().numpy()
                xyxy = boxes.xyxy
                
                for box in xyxy:
                    x_center = (box[0] + box[2]) / 2  # Calculate the center x-coordinate of the box
                    if x_center < frame_third:
                        position = "left"
                    elif x_center < 2 * frame_third:
                        position = "middle"
                    else:
                        position = "right"
                    
                    print(f"Box center at x={x_center}: {position}")

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
