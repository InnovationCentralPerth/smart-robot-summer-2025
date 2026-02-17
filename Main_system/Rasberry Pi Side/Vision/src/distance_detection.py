#!/usr/bin/env python3

import cv2
import depthai as dai
import blobconverter
import numpy as np
import time
import argparse
import os
import sys

# Parse arguments
parser = argparse.ArgumentParser()
parser.add_argument("-headless", action="store_true", help="Run in headless mode (no GUI).")
args = parser.parse_args()

# MobileNetSSD label texts (20 classes)
labelMap = ["background", "aeroplane", "bicycle", "bird", "boat", "bottle", "bus", "car", "cat", "chair", "cow",
            "diningtable", "dog", "horse", "motorbike", "person", "pottedplant", "sheep", "sofa", "train", "tvmonitor"]

print("Loading AI Model...")
custom_blob_path = "/home/icp/icp/Rasberry Pi Side/Vision/models/custom_model.blob"
custom_tflite_path = "/home/icp/icp/Rasberry Pi Side/Vision/models/custom_model.tflite"

nnBlobPath = ""

if os.path.exists(custom_blob_path):
    print(f"USING CUSTOM BLOB: {custom_blob_path}")
    nnBlobPath = custom_blob_path
else:
    print("Downloading default MobileNet SSD...")
    try:
        nnBlobPath = blobconverter.from_zoo(name="mobilenet-ssd", shaves=5)
    except Exception as e:
        print(f"Error downloading model: {e}")
        sys.exit(1)

print("Initializing pipeline...")
# Create pipeline
pipeline = dai.Pipeline()

# Define sources and outputs
camRgb = pipeline.create(dai.node.ColorCamera)
spatialDetectionNetwork = pipeline.create(dai.node.MobileNetSpatialDetectionNetwork)
monoLeft = pipeline.create(dai.node.MonoCamera)
monoRight = pipeline.create(dai.node.MonoCamera)
stereo = pipeline.create(dai.node.StereoDepth)

xoutRgb = pipeline.create(dai.node.XLinkOut)
xoutNN = pipeline.create(dai.node.XLinkOut)
xoutDepth = pipeline.create(dai.node.XLinkOut)

xoutRgb.setStreamName("rgb")
xoutNN.setStreamName("detections")
xoutDepth.setStreamName("depth")

# Aggressive USB Optimization
xoutRgb.input.setBlocking(False)
xoutRgb.input.setQueueSize(1)
xoutNN.input.setBlocking(False)
xoutNN.input.setQueueSize(1)
xoutDepth.input.setBlocking(False)
xoutDepth.input.setQueueSize(1)

# Properties
camRgb.setPreviewSize(300, 300)
camRgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
camRgb.setInterleaved(False)
camRgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)
camRgb.setFps(20)

monoLeft.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
monoLeft.setCamera("left")
monoLeft.setFps(20)

monoRight.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
monoRight.setCamera("right")
monoRight.setFps(20)

# Setting node configs
stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_DENSITY)
stereo.setDepthAlign(dai.CameraBoardSocket.CAM_A)
stereo.setOutputSize(monoLeft.getResolutionWidth(), monoLeft.getResolutionHeight())
stereo.setSubpixel(True)

spatialDetectionNetwork.setBlobPath(nnBlobPath)
spatialDetectionNetwork.setConfidenceThreshold(0.5)
spatialDetectionNetwork.input.setBlocking(False)
spatialDetectionNetwork.setBoundingBoxScaleFactor(0.5)
spatialDetectionNetwork.setDepthLowerThreshold(100)
spatialDetectionNetwork.setDepthUpperThreshold(5000)

# Linking
monoLeft.out.link(stereo.left)
monoRight.out.link(stereo.right)

camRgb.preview.link(spatialDetectionNetwork.input)
spatialDetectionNetwork.passthrough.link(xoutRgb.input)
spatialDetectionNetwork.out.link(xoutNN.input)

stereo.depth.link(spatialDetectionNetwork.inputDepth)
spatialDetectionNetwork.passthroughDepth.link(xoutDepth.input)

print("Starting device...")
with dai.Device(pipeline, maxUsbSpeed=dai.UsbSpeed.HIGH) as device:

    previewQueue = device.getOutputQueue(name="rgb", maxSize=1, blocking=False)
    detectionNNQueue = device.getOutputQueue(name="detections", maxSize=1, blocking=False)
    depthQueue = device.getOutputQueue(name="depth", maxSize=1, blocking=False)
    
    startTime = time.monotonic()
    counter = 0
    fps = 0
    color = (255, 255, 255)

    print("Running! Press 'q' to quit.")

    while True:
        time.sleep(0.001)
        
        inPreview = previewQueue.get()
        inDet = detectionNNQueue.get()
        depth = depthQueue.get()

        if not args.headless:
            counter+=1
            current_time = time.monotonic()
            if (current_time - startTime) > 1 :
                fps = counter / (current_time - startTime)
                counter = 0
                startTime = current_time

            frame = inPreview.getCvFrame()
            depthFrame = depth.getFrame()

            # Visualizing depth map
            depthFrameColor = cv2.normalize(depthFrame, None, 255, 0, cv2.NORM_INF, cv2.CV_8UC1)
            depthFrameColor = cv2.equalizeHist(depthFrameColor)
            depthFrameColor = cv2.applyColorMap(depthFrameColor, cv2.COLORMAP_HOT)

            detections = inDet.detections
            height, width = frame.shape[:2]

            # Find the NEAREST detection
            min_dist = 99999
            best_detection = None

            for detection in detections:
                dist = detection.spatialCoordinates.z
                if dist > 100 and dist < min_dist:
                    min_dist = dist
                    best_detection = detection

            if best_detection is not None:
                d = best_detection
                x1, x2 = int(d.xmin * width), int(d.xmax * width)
                y1, y2 = int(d.ymin * height), int(d.ymax * height)
                
                try: label = labelMap[d.label]
                except: label = f"ID:{d.label}"

                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 3)
                cv2.putText(frame, f"NEAREST: {label}", (x1 + 10, y1 + 20), cv2.FONT_HERSHEY_TRIPLEX, 0.5, (0,0,0), 3)
                cv2.putText(frame, f"NEAREST: {label}", (x1 + 10, y1 + 20), cv2.FONT_HERSHEY_TRIPLEX, 0.5, (0, 255, 0), 1)
                
                dist_str = f"{int(d.spatialCoordinates.z)} mm"
                cv2.putText(frame, dist_str, (x1 + 10, y1 + 40), cv2.FONT_HERSHEY_TRIPLEX, 0.5, (0,0,0), 3)
                cv2.putText(frame, dist_str, (x1 + 10, y1 + 40), cv2.FONT_HERSHEY_TRIPLEX, 0.5, (0, 255, 0), 1)

            # --- RANGEFINDER ---
            rgb_h, rgb_w = frame.shape[:2]
            cx, cy = int(rgb_w / 2), int(rgb_h / 2)
            dh, dw = depthFrame.shape[:2]
            dcx, dcy = int(dw / 2), int(dh / 2)
            
            roi = depthFrame[dcy-2:dcy+2, dcx-2:dcx+2]
            valid_pixels = roi[roi > 0]
            if len(valid_pixels) > 0:
                center_dist = int(np.mean(valid_pixels))
                dist_text = f"CENTER: {center_dist} mm"
                cross_color = (0, 255, 255)
            else:
                dist_text = "CENTER: TOO CLOSE"
                cross_color = (0, 0, 255)

            cv2.line(frame, (cx - 11, cy), (cx + 11, cy), (0,0,0), 4)
            cv2.line(frame, (cx, cy - 11), (cx, cy + 11), (0,0,0), 4)
            cv2.line(frame, (cx - 10, cy), (cx + 10, cy), (0, 255, 0), 2)
            cv2.line(frame, (cx, cy - 10), (cx, cy + 10), (0, 255, 0), 2)
            cv2.putText(frame, dist_text, (cx + 16, cy + 1), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,0), 2)
            cv2.putText(frame, dist_text, (cx + 15, cy), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

            cv2.putText(frame, "NN fps: {:.2f}".format(fps), (2, frame.shape[0] - 4), cv2.FONT_HERSHEY_TRIPLEX, 0.4, (255,255,255))
            cv2.imshow("depth", depthFrameColor)
            cv2.imshow("preview", frame)

            if cv2.waitKey(1) == ord('q'):
                break
