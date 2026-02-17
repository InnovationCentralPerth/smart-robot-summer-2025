import depthai as dai
import cv2
import time

pipeline = dai.Pipeline()
camRgb = pipeline.create(dai.node.ColorCamera)
xoutRgb = pipeline.create(dai.node.XLinkOut)
xoutRgb.setStreamName("rgb")
camRgb.preview.link(xoutRgb.input)

print("Starting camera test...")
with dai.Device(pipeline) as device:
    print("Device started!")
    qRgb = device.getOutputQueue(name="rgb", maxSize=1, blocking=False)
    while True:
        inRgb = qRgb.get()
        cv2.imshow("Test", inRgb.getCvFrame())
        if cv2.waitKey(1) == ord('q'):
            break
