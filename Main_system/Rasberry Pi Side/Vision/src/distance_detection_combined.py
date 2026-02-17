#!/usr/bin/env python3

import cv2
import depthai as dai
import blobconverter
import numpy as np
import time
import argparse
import os
import sys
import ai_edge_litert.interpreter as litert

# --- CONFIGURATION ---
# Default OAK-D Model (MobileNet SSD - 20 classes)
labelMapOAK = ["background", "aeroplane", "bicycle", "bird", "boat", "bottle", "bus", "car", "cat", "chair", "cow",
            "diningtable", "dog", "horse", "motorbike", "person", "pottedplant", "sheep", "sofa", "train", "tvmonitor"]

# Custom TFLite Model (3 classes) - UPDATE THESE
labelMapCustom = ["Custom Item 1", "Custom Item 2", "Custom Item 3"]
model_path = "/home/icp/icp/Rasberry Pi Side/Vision/models/custom_model.tflite"

# --- TFLITE SETUP (PI CPU) ---
print(f"Loading custom TFLite model: {model_path}")
interpreter = litert.Interpreter(model_path=model_path)
interpreter.allocate_tensors()
input_details = interpreter.get_input_details()
output_details = interpreter.get_output_details()
tflite_input_shape = input_details[0]['shape'] # e.g. [1, 96, 96, 3]

# --- OAK-D PIPELINE SETUP (VPU) ---
print("Initializing OAK-D Pipeline...")
pipeline = dai.Pipeline()

# Nodes
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

# Properties
camRgb.setPreviewSize(300, 300) # Match MobileNet
camRgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
camRgb.setInterleaved(False)
camRgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)
camRgb.setFps(15)

monoLeft.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
monoLeft.setCamera("left")
monoRight.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
monoRight.setCamera("right")

# Stereo Config
stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_ACCURACY)
stereo.setDepthAlign(dai.CameraBoardSocket.CAM_A)
stereo.setOutputSize(640, 400)

# OAK-D AI Model (MobileNet SSD)
nnBlobPath = blobconverter.from_zoo(name="mobilenet-ssd", shaves=5)
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

# --- START DEVICE ---
print("Starting Camera...")
with dai.Device(pipeline, maxUsbSpeed=dai.UsbSpeed.HIGH) as device:
    qRgb = device.getOutputQueue(name="rgb", maxSize=1, blocking=False)
    qDet = device.getOutputQueue(name="detections", maxSize=1, blocking=False)
    qDepth = device.getOutputQueue(name="depth", maxSize=1, blocking=False)

    startTime = time.monotonic()
    counter = 0
    fps = 0

    print("COMBINED MODE RUNNING!")
    print("Detecting standard objects (VPU) + Custom items (CPU).")

    while True:
        inRgb = qRgb.get()
        inDet = qDet.get()
        inDepth = qDepth.get()

        if inRgb is not None:
            frame = inRgb.getCvFrame() # 300x300 BGR
            depthFrame = inDepth.getFrame()
            detections = inDet.detections
            height, width = frame.shape[:2]

            # 1. RUN CUSTOM INFERENCE (CPU)
            # Resize 300x300 to model input (e.g. 96x96)
            tflite_frame = cv2.resize(frame, (tflite_input_shape[1], tflite_input_shape[2]))
            tflite_frame = cv2.cvtColor(tflite_frame, cv2.COLOR_BGR2RGB)
            input_data = np.expand_dims(tflite_frame, axis=0).astype(np.float32)
            input_data /= 255.0
            
            interpreter.set_tensor(input_details[0]['index'], input_data)
            interpreter.invoke()
            output_data = interpreter.get_tensor(output_details[0]['index'])[0]
            custom_top_idx = np.argmax(output_data)
            custom_conf = output_data[custom_top_idx]

            # 2. DRAW OAK-D DETECTIONS (Find Nearest)
            min_dist = 99999
            best_oak_det = None
            for d in detections:
                if d.spatialCoordinates.z > 100 and d.spatialCoordinates.z < min_dist:
                    min_dist = d.spatialCoordinates.z
                    best_oak_det = d

            if best_oak_det:
                x1, x2 = int(best_oak_det.xmin * width), int(best_oak_det.xmax * width)
                y1, y2 = int(best_oak_det.ymin * height), int(best_oak_det.ymax * height)
                try: label = labelMapOAK[best_oak_det.label]
                except: label = f"ID:{best_oak_det.label}"
                
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.putText(frame, f"OAK: {label}", (x1 + 5, y1 + 15), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

            # 3. RANGEFINDER (Center Distance)
            dh, dw = depthFrame.shape[:2]
            dcx, dcy = int(dw / 2), int(dh / 2)
            roi = depthFrame[dcy-2:dcy+2, dcx-2:dcx+2]
            valid_pixels = roi[roi > 0]
            center_dist = int(np.mean(valid_pixels)) if len(valid_pixels) > 0 else 0

            # 4. VISUALIZATION
            # Upscale for better viewing
            display_frame = cv2.resize(frame, (600, 600))
            cx, cy = 300, 300 # Center of 600x600

            # Overlay Custom Classification
            custom_label = labelMapCustom[custom_top_idx] if custom_conf > 0.6 else "Unknown"
            cv2.rectangle(display_frame, (0, 0), (600, 40), (0,0,0), -1)
            cv2.putText(display_frame, f"CUSTOM: {custom_label} ({custom_conf:.0%})", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)

            # Overlay Distance
            dist_text = f"DIST: {center_dist} mm"
            cv2.putText(display_frame, dist_text, (cx + 15, cy), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
            cv2.circle(display_frame, (cx, cy), 5, (0, 255, 255), -1)

            cv2.imshow("Combined AI Detector", display_frame)

        if cv2.waitKey(1) == ord('q'):
            break
