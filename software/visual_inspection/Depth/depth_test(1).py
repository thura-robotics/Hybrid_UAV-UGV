#!/usr/bin/env python3
import cv2
import numpy as np 
import depthai as dai

pipeline = dai.Pipeline()

# =========================
# (1) Cameras — use dai.node.Camera with CAM_B/CAM_C
# =========================
monoLeft  = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_B)
monoRight = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_C)

# =========================
# (2) Stereo Depth
# =========================
stereo = pipeline.create(dai.node.StereoDepth)

# Request outputs and link
monoLeftOut  = monoLeft.requestFullResolutionOutput()
monoRightOut = monoRight.requestFullResolutionOutput()
monoLeftOut.link(stereo.left)
monoRightOut.link(stereo.right)

# =========================
# (3) Preset
# =========================
stereo.setDefaultProfilePreset(
    dai.node.StereoDepth.PresetMode.HIGH_DETAIL
)

# =========================
# (4) Accuracy Features
# =========================
stereo.setLeftRightCheck(True)
stereo.setSubpixel(True)
stereo.setExtendedDisparity(False)

# =========================
# (5) Filters & Threshold
# =========================
stereo.initialConfig.setMedianFilter(dai.MedianFilter.KERNEL_7x7)
stereo.initialConfig.setConfidenceThreshold(200)

# =========================
# (6) Output — v3 inline queue, no XLinkOut needed
# =========================
depthQueue = stereo.depth.createOutputQueue()

# =========================
# (7) Run
# =========================
with pipeline:
    pipeline.start()
    while pipeline.isRunning():
        depthFrame = depthQueue.get()
        assert isinstance(depthFrame, dai.ImgFrame)
        cv2.imshow("depth", depthFrame.getCvFrame())
        if cv2.waitKey(1) == ord("q"):
            break