#!/usr/bin/env python3

import cv2
import depthai as dai
import blobconverter
import numpy as np
import time
import socket
import struct
import json

# CONFIGURATION
# Default to 192.168.6.11 if not passed as arg, but allow override
import sys
if len(sys.argv) > 1:
    UDP_IP = sys.argv[1]
else:
    UDP_IP = "192.168.6.11" # Your PC IP

UDP_PORT = 5005
FRAME_SIZE = 416

# Socket Setup
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# YOLOv3 Tiny label texts (COCO 80 classes)
labelMap = [
    "person", "bicycle", "car", "motorbike", "aeroplane", "bus", "train", "truck", "boat", "traffic light", "fire hydrant", "stop sign", "parking meter", "bench", "bird", "cat", "dog", "horse", "sheep", "cow", "elephant", "bear", "zebra", "giraffe", "backpack", "umbrella", "handbag", "tie", "suitcase", "frisbee", "skis", "snowboard", "sports ball", "kite", "baseball bat", "baseball glove", "skateboard", "surfboard", "tennis racket", "bottle", "wine glass", "cup", "fork", "knife", "spoon", "bowl", "banana", "apple", "sandwich", "orange", "broccoli", "carrot", "hot dog", "pizza", "donut", "cake", "chair", "sofa", "pottedplant", "bed", "diningtable", "toilet", "tvmonitor", "laptop", "mouse", "remote", "keyboard", "cell phone", "microwave", "oven", "toaster", "sink", "refrigerator", "book", "clock", "vase", "scissors", "teddy bear", "hair drier", "toothbrush"
]

print("Initializing pipeline...")
pipeline = dai.Pipeline()

# Nodes
camRgb = pipeline.create(dai.node.ColorCamera)
spatialDetectionNetwork = pipeline.create(dai.node.YoloSpatialDetectionNetwork)
monoLeft = pipeline.create(dai.node.MonoCamera)
monoRight = pipeline.create(dai.node.MonoCamera)
stereo = pipeline.create(dai.node.StereoDepth)

# Outputs
xoutRgb = pipeline.create(dai.node.XLinkOut)
xoutNN = pipeline.create(dai.node.XLinkOut)
xoutDepth = pipeline.create(dai.node.XLinkOut) # Enable Depth Stream

xoutRgb.setStreamName("rgb")
xoutNN.setStreamName("detections")
xoutDepth.setStreamName("depth") # Enable Depth Stream

# Optimization: Limit USB bandwidth
xoutRgb.input.setBlocking(False)
xoutRgb.input.setQueueSize(1)
xoutNN.input.setBlocking(False)
xoutNN.input.setQueueSize(1)
xoutDepth.input.setBlocking(False)
xoutDepth.input.setQueueSize(1)

# Properties
camRgb.setPreviewSize(FRAME_SIZE, FRAME_SIZE)
camRgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
camRgb.setInterleaved(False)
camRgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)
camRgb.setFps(10) # Lower FPS for stability

monoLeft.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
monoLeft.setCamera("left")
monoLeft.setFps(10)

monoRight.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
monoRight.setCamera("right")
monoRight.setFps(10)

# Stereo Config (High Accuracy / No Subpixel for stability)
stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_ACCURACY)
stereo.setDepthAlign(dai.CameraBoardSocket.CAM_A)
stereo.setOutputSize(monoLeft.getResolutionWidth(), monoLeft.getResolutionHeight())
stereo.setSubpixel(False)

# AI Model
print("Loading YOLOv3...")
# Reduce to 4 Shaves to prevent crash
nnBlobPath = blobconverter.from_zoo(name="yolo-v3-tiny-tf", shaves=4, version="2021.4")
spatialDetectionNetwork.setBlobPath(nnBlobPath)
spatialDetectionNetwork.setConfidenceThreshold(0.5)
spatialDetectionNetwork.input.setBlocking(False)
spatialDetectionNetwork.setBoundingBoxScaleFactor(0.5)
spatialDetectionNetwork.setDepthLowerThreshold(100)
spatialDetectionNetwork.setDepthUpperThreshold(10000)

# Yolo Config
spatialDetectionNetwork.setNumClasses(80)
spatialDetectionNetwork.setCoordinateSize(4)
spatialDetectionNetwork.setAnchors([10,14, 23,27, 37,58, 81,82, 135,169, 344,319])
spatialDetectionNetwork.setAnchorMasks({ "side26": [1,2,3], "side13": [3,4,5] })
spatialDetectionNetwork.setIouThreshold(0.5)

# Linking
monoLeft.out.link(stereo.left)
monoRight.out.link(stereo.right)
camRgb.preview.link(spatialDetectionNetwork.input)
spatialDetectionNetwork.passthrough.link(xoutRgb.input) # Send frames to host
spatialDetectionNetwork.out.link(xoutNN.input) # Send detections to host
stereo.depth.link(spatialDetectionNetwork.inputDepth)
spatialDetectionNetwork.passthroughDepth.link(xoutDepth.input) # Link Depth to XLinkOut

print("Starting streaming...")
with dai.Device(pipeline, maxUsbSpeed=dai.UsbSpeed.HIGH) as device:
    qRgb = device.getOutputQueue(name="rgb", maxSize=1, blocking=False)
    qDet = device.getOutputQueue(name="detections", maxSize=1, blocking=False)
    qDepth = device.getOutputQueue(name="depth", maxSize=1, blocking=False)

    print(f"Streaming Video to {UDP_IP}:{UDP_PORT} and Depth to {UDP_PORT+1}")

    while True:
        inRgb = qRgb.get()
        inDet = qDet.get()
        inDepth = qDepth.get()

        if inRgb is not None:
            frame = inRgb.getCvFrame()
            detections = inDet.detections
            
            # Process Depth Frame
            depthFrame = inDepth.getFrame()
            depthFrameColor = cv2.normalize(depthFrame, None, 255, 0, cv2.NORM_INF, cv2.CV_8UC1)
            depthFrameColor = cv2.equalizeHist(depthFrameColor)
            depthFrameColor = cv2.applyColorMap(depthFrameColor, cv2.COLORMAP_HOT)
            
            # Resize depth to match RGB just in case, though they should match
            depthFrameColor = cv2.resize(depthFrameColor, (FRAME_SIZE, FRAME_SIZE))

            # Find best detection
            best = None
            max_conf = 0
            for d in detections:
                if d.confidence > max_conf:
                    max_conf = d.confidence
                    best = d

            # Draw ONLY the best detection
            if best:
                x1 = int(best.xmin * FRAME_SIZE)
                y1 = int(best.ymin * FRAME_SIZE)
                x2 = int(best.xmax * FRAME_SIZE)
                y2 = int(best.ymax * FRAME_SIZE)

                try:
                    label = labelMap[best.label]
                except:
                    label = str(best.label)

                conf = f"{int(best.confidence * 100)}%"
                dist = f"{int(best.spatialCoordinates.z)}mm"

                # Draw Box
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                
                # Draw Text Background
                cv2.rectangle(frame, (x1, y1-30), (x2, y1), (0, 255, 0), -1)
                cv2.putText(frame, f"{label} {dist}", (x1 + 5, y1 - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 2)

            # --- UNIVERSAL RANGEFINDER MODE (Remote Stream) ---
            center_x = int(depthFrame.shape[1] / 2)
            center_y = int(depthFrame.shape[0] / 2)
            
            # Small ROI
            roi = depthFrame[center_y-2:center_y+2, center_x-2:center_x+2]
            
            # Avg distance
            valid_pixels = roi[roi > 0]
            if len(valid_pixels) > 0:
                center_dist = int(np.mean(valid_pixels))
                dist_text = f"CENTER: {center_dist} mm"
                color = (0, 255, 255) # Yellow
            else:
                dist_text = "CENTER: TOO CLOSE"
                color = (0, 0, 255) # Red

            # Draw Crosshair
            cv2.line(frame, (center_x - 10, center_y), (center_x + 10, center_y), color, 2)
            cv2.line(frame, (center_x, center_y - 10), (center_x, center_y + 10), color, 2)
            cv2.putText(frame, dist_text, (center_x + 15, center_y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
            # --------------------------------------------------

            # Compress Frames to JPEG
            encoded_rgb, buffer_rgb = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 50])
            encoded_depth, buffer_depth = cv2.imencode('.jpg', depthFrameColor, [cv2.IMWRITE_JPEG_QUALITY, 50])
            
            # Send RGB over UDP Port 5005
            if len(buffer_rgb) < 65000:
                sock.sendto(buffer_rgb, (UDP_IP, UDP_PORT))
                
            # Send Depth over UDP Port 5006
            if len(buffer_depth) < 65000:
                sock.sendto(buffer_depth, (UDP_IP, UDP_PORT + 1))

        # 20 FPS limiter
        time.sleep(0.05)
