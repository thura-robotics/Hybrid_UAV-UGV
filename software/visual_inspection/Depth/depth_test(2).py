#!/usr/bin/env python3

import cv2
import depthai as dai
import numpy as np

pipeline = dai.Pipeline()

# Cameras
monoLeft  = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_B)
monoRight = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_C)

stereo = pipeline.create(dai.node.StereoDepth)

# Linking
monoLeftOut  = monoLeft.requestFullResolutionOutput()
monoRightOut = monoRight.requestFullResolutionOutput()
monoLeftOut.link(stereo.left)
monoRightOut.link(stereo.right)

# Stereo settings
stereo.setLeftRightCheck(True)
stereo.setExtendedDisparity(True)
stereo.setSubpixel(False)

# Outputs
dispQueue  = stereo.disparity.createOutputQueue()
depthQueue = stereo.depth.createOutputQueue()

# Colormap
colorMap = cv2.applyColorMap(
    np.arange(256, dtype=np.uint8),
    cv2.COLORMAP_JET
)

with pipeline:
    pipeline.start()

    maxDisp = 1

    while pipeline.isRunning():
        inDisp  = dispQueue.get()
        inDepth = depthQueue.get()

        dispFrame  = inDisp.getFrame()
        depthFrame = inDepth.getFrame()

        # ----------------------------
        # 🎨 Disparity visualization
        # ----------------------------
        maxDisp = max(maxDisp, np.max(dispFrame))
        dispNorm = ((dispFrame / maxDisp) * 255).astype(np.uint8)
        dispColor = cv2.applyColorMap(dispNorm, colorMap)

        # ----------------------------
        # 📏 Depth measurement (center)
        # ----------------------------
        h, w = depthFrame.shape
        cx, cy = w // 2, h // 2

        # small ROI around center
        roi = depthFrame[cy-10:cy+10, cx-10:cx+10]
        roi = roi[roi > 0]   # remove invalid pixels

        if len(roi) > 0:
            distance = int(np.median(roi))
            text = f"{distance} mm"
        else:
            text = "N/A"

        # ----------------------------
        # 🧩 Overlay text on disparity
        # ----------------------------
        cv2.circle(dispColor, (cx, cy), 5, (255,255,255), -1)
        cv2.putText(dispColor, text, (cx+10, cy-10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255,255,255), 2)

        # ----------------------------
        # Show result
        # ----------------------------
        cv2.imshow("Hybrid View (Disparity + Distance)", dispColor)

        if cv2.waitKey(1) == ord('q'):
            pipeline.stop()
            break