#!/usr/bin/env python3

import cv2
import depthai as dai
import blobconverter
import numpy as np
import time
import argparse

# Parse arguments
parser = argparse.ArgumentParser()
parser.add_argument("-headless", action="store_true", help="Run in headless mode (no GUI).")
args = parser.parse_args()

# YOLOv3 Tiny label texts (COCO 80 classes)
labelMap = [
    "person", "bicycle", "car", "motorbike", "aeroplane", "bus", "train", "truck", "boat", "traffic light", "fire hydrant", "stop sign", "parking meter", "bench", "bird", "cat", "dog", "horse", "sheep", "cow", "elephant", "bear", "zebra", "giraffe", "backpack", "umbrella", "handbag", "tie", "suitcase", "frisbee", "skis", "snowboard", "sports ball", "kite", "baseball bat", "baseball glove", "skateboard", "surfboard", "tennis racket", "bottle", "wine glass", "cup", "fork", "knife", "spoon", "bowl", "banana", "apple", "sandwich", "orange", "broccoli", "carrot", "hot dog", "pizza", "donut", "cake", "chair", "sofa", "pottedplant", "bed", "diningtable", "toilet", "tvmonitor", "laptop", "mouse", "remote", "keyboard", "cell phone", "microwave", "oven", "toaster", "sink", "refrigerator", "book", "clock", "vase", "scissors", "teddy bear", "hair drier", "toothbrush"
]

print("Initializing pipeline...")
# Create pipeline
pipeline = dai.Pipeline()

# Define sources and outputs
camRgb = pipeline.create(dai.node.ColorCamera)
spatialDetectionNetwork = pipeline.create(dai.node.YoloSpatialDetectionNetwork)
monoLeft = pipeline.create(dai.node.MonoCamera)
monoRight = pipeline.create(dai.node.MonoCamera)
stereo = pipeline.create(dai.node.StereoDepth)

# OUTPUTS: Only stream Detections (Metadata) to save massive bandwidth/processing
# xoutRgb = pipeline.create(dai.node.XLinkOut)
xoutNN = pipeline.create(dai.node.XLinkOut)

# xoutRgb.setStreamName("rgb")
xoutNN.setStreamName("detections")

# Properties
camRgb.setPreviewSize(416, 416)
camRgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
camRgb.setInterleaved(False)
camRgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)
camRgb.setFps(10) # Low FPS for stability

monoLeft.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
monoLeft.setCamera("left")
monoLeft.setFps(10)

monoRight.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
monoRight.setCamera("right")
monoRight.setFps(10)

# Setting node configs
stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_DENSITY)
stereo.setDepthAlign(dai.CameraBoardSocket.CAM_A)
stereo.setOutputSize(monoLeft.getResolutionWidth(), monoLeft.getResolutionHeight())
stereo.setSubpixel(True)

print("Downloading/Loading YOLOv3 Tiny (80 classes)...")
try:
    nnBlobPath = blobconverter.from_zoo(name="yolo-v3-tiny-tf", shaves=5, version="2021.4")
except Exception as e:
    print(f"Error downloading model: {e}")
    import sys
    sys.exit(1)

spatialDetectionNetwork.setBlobPath(nnBlobPath)
spatialDetectionNetwork.setConfidenceThreshold(0.5)
spatialDetectionNetwork.input.setBlocking(False)
spatialDetectionNetwork.setBoundingBoxScaleFactor(0.5)
spatialDetectionNetwork.setDepthLowerThreshold(100)
spatialDetectionNetwork.setDepthUpperThreshold(5000)

# Yolo specific parameters
spatialDetectionNetwork.setNumClasses(80)
spatialDetectionNetwork.setCoordinateSize(4)
spatialDetectionNetwork.setAnchors([10,14, 23,27, 37,58, 81,82, 135,169, 344,319])
spatialDetectionNetwork.setAnchorMasks({ "side26": [1,2,3], "side13": [3,4,5] })
spatialDetectionNetwork.setIouThreshold(0.5)

# Linking
monoLeft.out.link(stereo.left)
monoRight.out.link(stereo.right)

camRgb.preview.link(spatialDetectionNetwork.input)
# spatialDetectionNetwork.passthrough.link(xoutRgb.input)
spatialDetectionNetwork.out.link(xoutNN.input)

stereo.depth.link(spatialDetectionNetwork.inputDepth)

import socket
import json

# Setup UDP Socket for Remote Data Streaming
UDP_IP = "192.168.6.11"  # Target PC IP
UDP_PORT = 5005
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

print(f"Streaming detection data to {UDP_IP}:{UDP_PORT}")

print("Starting device in HEADLESS MODE (No Video Stream)...")
# Connect to device and start pipeline
with dai.Device(pipeline, maxUsbSpeed=dai.UsbSpeed.HIGH) as device:

    # Output queues
    detectionNNQueue = device.getOutputQueue(name="detections", maxSize=1, blocking=False)
    
    print("Running! Sending data to PC...")

    while True:
        time.sleep(0.001)
        
        inDet = detectionNNQueue.get()
        detections = inDet.detections

        if len(detections) > 0:
            # Find highest confidence
            max_conf = 0
            best = None
            for d in detections:
                if d.confidence > max_conf:
                    max_conf = d.confidence
                    best = d
            
            if best:
                try:
                    label = labelMap[best.label]
                except:
                    label = str(best.label)
                
                # Prepare data packet
                data = {
                    "object": label,
                    "confidence": round(best.confidence * 100, 1),
                    "distance_mm": int(best.spatialCoordinates.z),
                    "x_pos": int(best.spatialCoordinates.x)
                }
                
                # Send to PC via UDP
                message = json.dumps(data).encode('utf-8')
                sock.sendto(message, (UDP_IP, UDP_PORT))
                
                # Also print locally for debugging
                print(f"SENT: {label} | {data['distance_mm']}mm")


