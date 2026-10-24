import asyncio
import cv2
import av
import numpy as np
import uuid
import depthai as dai

from aiohttp import web
from aiortc import RTCPeerConnection, RTCSessionDescription, VideoStreamTrack

pcs = set()

# ======================================
# OAK-D LITE SHARED CAMERA
# ======================================

class SharedCamera:
    def __init__(self):
        self.frame = None
        self._lock = asyncio.Lock()
        self._task = None

        print("[OAK-D] Initializing pipeline")

    def start(self):
        self._task = asyncio.ensure_future(self._read_loop())
        print("[OAK-D] Background read loop started")

    async def _read_loop(self):
        loop = asyncio.get_event_loop()

        try:
            # DepthAI v3: build pipeline inside Device context
            with dai.Device() as device:

                print("[OAK-D] Device connected")

                # v3 API: create pipeline on the device directly
                pipeline = dai.Pipeline(device)

                # Camera node - use 'CAM_A' string for RGB on OAK-D Lite
                cam = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_A)

                # Get output queue directly from camera node (no XLinkOut needed in v3)
                video_queue = cam.requestOutput(
                    size=(1280, 720),
                    type=dai.ImgFrame.Type.BGR888p
                )

                pipeline.start()

                while pipeline.isRunning():

                    img_frame = await loop.run_in_executor(
                        None, lambda: video_queue.get()
                    )

                    if img_frame is not None:
                        frame = img_frame.getCvFrame()

                        async with self._lock:
                            self.frame = frame

                    await asyncio.sleep(0.001)

        except Exception as e:
            print("[OAK-D] Error:", e)
            import traceback
            traceback.print_exc()

    async def get_frame(self):

        async with self._lock:
            frame = self.frame.copy() if self.frame is not None else None

        if frame is None:
            return np.zeros((720, 1280, 3), dtype="uint8")

        # Convert BGR → RGB for WebRTC
        return cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

    def stop(self):

        if self._task:
            self._task.cancel()

        print("[OAK-D] Camera stopped")


# ======================================
# WEBRTC VIDEO TRACK
# ======================================

class WebcamTrack(VideoStreamTrack):

    kind = "video"

    def __init__(self, camera: SharedCamera):
        super().__init__()
        self.camera = camera

    async def recv(self):

        pts, time_base = await self.next_timestamp()

        frame_rgb = await self.camera.get_frame()

        video_frame = av.VideoFrame.from_ndarray(
            frame_rgb,
            format="rgb24"
        )

        video_frame.pts = pts
        video_frame.time_base = time_base

        return video_frame


# ======================================
# WAIT FOR ICE
# ======================================

async def wait_for_ice(pc):

    if pc.iceGatheringState == "complete":
        return

    ice_complete = asyncio.Event()

    @pc.on("icegatheringstatechange")
    def on_ice_gathering_state_change():

        print("[ICE]", pc.iceGatheringState)

        if pc.iceGatheringState == "complete":
            ice_complete.set()

    try:
        await asyncio.wait_for(
            ice_complete.wait(),
            timeout=5.0
        )
    except asyncio.TimeoutError:
        print("[ICE] timeout")


# ======================================
# SERVER ROUTES
# ======================================

async def index(request):
    return web.FileResponse("./index.html")


async def offer(request):

    params = await request.json()

    pc = RTCPeerConnection()

    pcs.add(pc)

    print("[WebRTC] peer connected")

    # Unique stream id fix
    unique_stream_id = str(uuid.uuid4())
    pc._RTCPeerConnection__stream_id = unique_stream_id

    @pc.on("connectionstatechange")
    async def on_connectionstatechange():

        print("[WebRTC] state:", pc.connectionState)

        if pc.connectionState in ("failed", "closed"):

            await pc.close()
            pcs.discard(pc)

    track = WebcamTrack(shared_camera)

    pc.addTrack(track)

    await pc.setRemoteDescription(
        RTCSessionDescription(
            sdp=params["sdp"],
            type=params["type"]
        )
    )

    answer = await pc.createAnswer()

    await pc.setLocalDescription(answer)

    await wait_for_ice(pc)

    return web.json_response(
        {
            "sdp": pc.localDescription.sdp,
            "type": pc.localDescription.type
        }
    )


# ======================================
# SERVER LIFECYCLE
# ======================================

async def on_startup(app):

    print("[Server] starting camera")

    shared_camera.start()


async def on_shutdown(app):

    print("[Server] shutting down")

    coros = [pc.close() for pc in pcs]

    await asyncio.gather(*coros)

    pcs.clear()

    shared_camera.stop()


# ======================================
# START SERVER
# ======================================

shared_camera = SharedCamera()

app = web.Application()

app.on_startup.append(on_startup)
app.on_shutdown.append(on_shutdown)

app.router.add_get("/", index)
app.router.add_get("/index.html", index)
app.router.add_post("/offer", offer)

app.router.add_static("/", path="./", show_index=False)

if __name__ == "__main__":

    print("[Server] starting on http://0.0.0.0:8080")

    web.run_app(
        app,
        host="0.0.0.0",
        port=8080
    )