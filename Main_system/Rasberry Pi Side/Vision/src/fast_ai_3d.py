
import cv2
import depthai as dai
import blobconverter

# Create pipeline
pipeline = dai.Pipeline()

# Nodes
camRgb = pipeline.create(dai.node.ColorCamera)
faceDet = pipeline.create(dai.node.MobileNetDetectionNetwork) # Regular det, no spatial
xoutRgb = pipeline.create(dai.node.XLinkOut)
xoutNN = pipeline.create(dai.node.XLinkOut)

xoutRgb.setStreamName("rgb")
xoutNN.setStreamName("detections")

# Properties
camRgb.setPreviewSize(300, 300)
camRgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
camRgb.setInterleaved(False)
camRgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)
camRgb.setFps(30) # PURE SPEED

# Face Detection AI (Ultra light)
faceDet.setBlobPath(blobconverter.from_zoo(name="face-detection-retail-0004", shaves=5))
faceDet.setConfidenceThreshold(0.5)
faceDet.input.setBlocking(False)

# Linking
camRgb.preview.link(faceDet.input)
faceDet.passthrough.link(xoutRgb.input)
faceDet.out.link(xoutNN.input)

# Connect
# FORCE USB2.0 for stability
with dai.Device(pipeline, maxUsbSpeed=dai.UsbSpeed.HIGH) as device:
    qRgb = device.getOutputQueue(name="rgb", maxSize=4, blocking=False)
    qDet = device.getOutputQueue(name="detections", maxSize=4, blocking=False)

    print("Ultra-Fastest AI Mode Running (Face Track)")
    print("Press 'q' to quit.")

    while True:
        inRgb = qRgb.get()
        inDet = qDet.get()
        
        frame = inRgb.getCvFrame()
        detections = inDet.detections

        for d in detections:
            x1, y1 = int(d.xmin * 300), int(d.ymin * 300)
            x2, y2 = int(d.xmax * 300), int(d.ymax * 300)
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(frame, "FACE", (x1 + 5, y1 + 15), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0))

        cv2.imshow("Fastest AI Preview", frame)
        if cv2.waitKey(1) == ord('q'):
            break
