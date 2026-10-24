#!/usr/bin/env python3

import cv2
import depthai as dai
import time

# FPS variables
frame_count = 0
start_time = time.time()
fps = 0

# Create pipeline
with dai.Pipeline() as pipeline:
    cam = pipeline.create(dai.node.Camera).build()
    videoQueue = cam.requestOutput((640,400)).createOutputQueue()

    pipeline.start()

    while pipeline.isRunning():
        videoIn = videoQueue.get()
        frame = videoIn.getCvFrame()

        # Count frames
        frame_count += 1

        # Calculate FPS every 1 second
        current_time = time.time()
        elapsed_time = current_time - start_time

        if elapsed_time > 1.0:
            fps = frame_count / elapsed_time
            frame_count = 0
            start_time = current_time

        # Put FPS text on frame
        cv2.putText(frame, f"FPS: {fps:.2f}", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

        cv2.imshow("video", frame)

        if cv2.waitKey(1) == ord("q"):
            break

cv2.destroyAllWindows()