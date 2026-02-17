#!/usr/bin/env python3

import cv2
import depthai as dai
import numpy as np
import time
import argparse
import os
import ai_edge_litert.interpreter as litert

# Labels (Change these to match your Edge Impulse classes!)
labelMap = ["Item 1", "Item 2", "Item 3"]

# Model path
model_path = "/home/icp/icp/Rasberry Pi Side/Vision/models/custom_model.tflite"

print(f"Loading custom TFLite model: {model_path}")
interpreter = litert.Interpreter(model_path=model_path)
interpreter.allocate_tensors()
input_details = interpreter.get_input_details()
output_details = interpreter.get_output_details()
input_shape = input_details[0]['shape'] # [1, 96, 96, 3]

print("Initializing Pipeline...")
pipeline = dai.Pipeline()

# Nodes
camRgb = pipeline.create(dai.node.ColorCamera)
monoLeft = pipeline.create(dai.node.MonoCamera)
monoRight = pipeline.create(dai.node.MonoCamera)
stereo = pipeline.create(dai.node.StereoDepth)

xoutRgb = pipeline.create(dai.node.XLinkOut)
xoutDepth = pipeline.create(dai.node.XLinkOut)

xoutRgb.setStreamName("rgb")
xoutDepth.setStreamName("depth")

# Properties
camRgb.setPreviewSize(input_shape[1], input_shape[2]) # Match model input (96x96)
camRgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
camRgb.setInterleaved(False)
camRgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.RGB) # TFLite models usually want RGB
camRgb.setFps(20)

monoLeft.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
monoLeft.setCamera("left")
monoRight.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
monoRight.setCamera("right")

# Stereo Config
stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_ACCURACY)
stereo.setDepthAlign(dai.CameraBoardSocket.CAM_A)
stereo.setOutputSize(640, 400) # Stable depth resolution

# Linking
camRgb.preview.link(xoutRgb.input)
monoLeft.out.link(stereo.left)
monoRight.out.link(stereo.right)
stereo.depth.link(xoutDepth.input)

print("Starting Camera...")
with dai.Device(pipeline, maxUsbSpeed=dai.UsbSpeed.HIGH) as device:
    qRgb = device.getOutputQueue(name="rgb", maxSize=1, blocking=False)
    qDepth = device.getOutputQueue(name="depth", maxSize=1, blocking=False)

    print("Running! Detecting custom items + Distance.")
    print("Press 'q' in the window to quit.")

    while True:
        inRgb = qRgb.get()
        inDepth = qDepth.get()

        if inRgb is not None:
            # 1. Get Frame
            frame = inRgb.getCvFrame() # 96x96 RGB
            depthFrame = inDepth.getFrame()

            # 2. RUN INFERENCE ON PI CPU
            # Prepare input data (Normalize 0.0 to 1.0)
            input_data = np.expand_dims(frame, axis=0).astype(np.float32)
            input_data /= 255.0
            
            interpreter.set_tensor(input_details[0]['index'], input_data)
            interpreter.invoke()
            
            # Get Results
            output_data = interpreter.get_tensor(output_details[0]['index'])[0]
            top_index = np.argmax(output_data)
            confidence = output_data[top_index]

            # 3. RANGEFINDER LOGIC
            dh, dw = depthFrame.shape[:2]
            dcx, dcy = int(dw / 2), int(dh / 2)
            roi = depthFrame[dcy-2:dcy+2, dcx-2:dcx+2]
            valid_pixels = roi[roi > 0]
            center_dist = int(np.mean(valid_pixels)) if len(valid_pixels) > 0 else 0

            # 4. VISUALIZATION
            # Upscale for display
            display_frame = cv2.resize(frame, (400, 400))
            display_frame = cv2.cvtColor(display_frame, cv2.COLOR_RGB2BGR)
            
            cx, cy = 200, 200 # Center of 400x400
            
            # Show Classification
            label = labelMap[top_index] if confidence > 0.6 else "Unknown"
            cv2.rectangle(display_frame, (0, 0), (400, 40), (0,0,0), -1)
            cv2.putText(display_frame, f"ITEM: {label} ({confidence:.0%})", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

            # Show Distance
            dist_text = f"DIST: {center_dist} mm" if center_dist > 0 else "DIST: TOO CLOSE"
            cv2.putText(display_frame, dist_text, (cx + 10, cy), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
            cv2.circle(display_frame, (cx, cy), 5, (0, 255, 255), -1)

            cv2.imshow("Custom Detector", display_frame)

        if cv2.waitKey(1) == ord('q'):
            break
