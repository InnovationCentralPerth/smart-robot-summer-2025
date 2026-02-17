#!/usr/bin/env python3

import depthai as dai
import numpy as np
import rerun as rr
import cv2
import time

# --- CONFIGURATION ---
FPS = 15
DEPTH_RES = (640, 400) # 400P
RGB_RES = (640, 400)   # Resized to match depth

# Initialize Rerun
print("Initializing Rerun Visualizer...")
rr.init("OAK-D_Lite_SLAM_3", spawn=True)

# Create Pipeline
print("Creating Pipeline...")
pipeline = dai.Pipeline()

# Nodes
camRgb = pipeline.create(dai.node.ColorCamera)
monoLeft = pipeline.create(dai.node.MonoCamera)
monoRight = pipeline.create(dai.node.MonoCamera)
stereo = pipeline.create(dai.node.StereoDepth)
sync = pipeline.create(dai.node.Sync)

# XLinkOut
xout = pipeline.create(dai.node.XLinkOut)
xout.setStreamName("sync")

# Properties
camRgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
camRgb.setPreviewSize(RGB_RES[0], RGB_RES[1])
camRgb.setInterleaved(False)
camRgb.setFps(FPS)
camRgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.RGB)

monoLeft.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
monoLeft.setCamera("left")
monoLeft.setFps(FPS)

monoRight.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
monoRight.setCamera("right")
monoRight.setFps(FPS)

# Stereo
stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_DENSITY)
stereo.setDepthAlign(dai.CameraBoardSocket.CAM_A)
stereo.setOutputSize(DEPTH_RES[0], DEPTH_RES[1])
stereo.setSubpixel(False) # Disable for stability

# Linking
monoLeft.out.link(stereo.left)
monoRight.out.link(stereo.right)
camRgb.preview.link(sync.inputs["rgb"])
stereo.depth.link(sync.inputs["depth"])
sync.out.link(xout.input)

# Connect (Force USB2)
print("Starting Camera...")
with dai.Device(pipeline, maxUsbSpeed=dai.UsbSpeed.HIGH) as device:
    q = device.getOutputQueue("sync", maxSize=1, blocking=False)
    
    calibData = device.readCalibration()
    intrinsics = calibData.getCameraIntrinsics(dai.CameraBoardSocket.CAM_A, RGB_RES[0], RGB_RES[1])
    
    print("Running! Check the Rerun window.")
    
    while True:
        inSync = q.get()
        frame = inSync["rgb"].getCvFrame()
        depth = inSync["depth"].getFrame()

        # Log to Rerun
        rr.log("world/camera", rr.Pinhole(
            resolution=[RGB_RES[0], RGB_RES[1]],
            image_from_camera=intrinsics
        ))
        
        # Log RGB
        rr.log("world/camera/rgb", rr.Image(frame))
        
        # Log Depth
        # Rerun handles depth visualization automatically
        rr.log("world/camera/depth", rr.DepthImage(depth, meter=1000.0))

        if cv2.waitKey(1) == ord('q'):
            break
