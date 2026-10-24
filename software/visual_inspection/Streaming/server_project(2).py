import asyncio, cv2, av, numpy as np, uuid
from aiohttp import web
from aiortc import RTCPeerConnection, RTCSessionDescription, VideoStreamTrack

pcs = set()


# ======================================
# GPU CHECK
# ======================================

def check_gpu():
    cuda_available = cv2.cuda.getCudaEnabledDeviceCount() > 0
    if cuda_available:
        print(f"[GPU] CUDA available — {cv2.cuda.getCudaEnabledDeviceCount()} device(s)")
    else:
        print("[GPU] CUDA not available — using CPU")
    return cuda_available


# ======================================
# SHARED CAMERA
# ======================================

class SharedCamera:
    
    def __init__(self):
        self.cap = None
        self.frame = None
        self.use_gpu = False
        self._lock = asyncio.Lock()
        self._task = None
        self._open_camera()

    def _open_camera(self):
        for index in range(5):
            cap = cv2.VideoCapture(index, cv2.CAP_DSHOW)
            if cap.isOpened():
                ret, frame = cap.read()
                if ret and frame is not None:
                    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
                    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
                    cap.set(cv2.CAP_PROP_FPS, 30)
                    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
                    self.cap = cap
                    self.frame = frame
                    print(f"[Camera] Opened at index {index} — 1280x720 @ 30fps")
                    return
            cap.release()
        raise RuntimeError("No working camera found on indices 0-4.")

    def start(self):
        self._task = asyncio.ensure_future(self._read_loop())
        print("[Camera] Background read loop started")

    async def _read_loop(self):
        loop = asyncio.get_event_loop()
        while True:
            ret, frame = await loop.run_in_executor(None, self.cap.read)
            if ret and frame is not None:
                async with self._lock:
                    self.frame = frame
            await asyncio.sleep(0)

    async def get_frame(self):
        async with self._lock:
            frame = self.frame.copy() if self.frame is not None else None
        if frame is None:
            return np.zeros((720, 1280, 3), dtype="uint8")
        if self.use_gpu:
            gpu_frame = cv2.cuda_GpuMat()
            gpu_frame.upload(frame)
            gpu_frame = cv2.cuda.cvtColor(gpu_frame, cv2.COLOR_BGR2RGB)
            return gpu_frame.download()
        else:
            return cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

    def stop(self):
        if self._task:
            self._task.cancel()
        if self.cap and self.cap.isOpened():
            self.cap.release()
            print("[Camera] Released")


# ======================================
# PER-CONNECTION TRACK
# ======================================

class WebcamTrack(VideoStreamTrack):
    kind = "video"

    def __init__(self, camera: SharedCamera):
        super().__init__()
        self.camera = camera

    async def recv(self):
        pts, time_base = await self.next_timestamp()
        frame = await self.camera.get_frame()
        video_frame = av.VideoFrame.from_ndarray(frame, format="rgb24")
        video_frame.pts = pts
        video_frame.time_base = time_base
        return video_frame


# ======================================
# WAIT FOR ICE GATHERING
# ======================================

async def wait_for_ice(pc):
    if pc.iceGatheringState == "complete":
        return

    ice_complete = asyncio.Event()

    @pc.on("icegatheringstatechange")
    def on_ice_gathering_state_change():
        print(f"[ICE] Gathering state: {pc.iceGatheringState}")
        if pc.iceGatheringState == "complete":
            ice_complete.set()

    try:
        await asyncio.wait_for(ice_complete.wait(), timeout=5.0)
        print("[ICE] Gathering complete")
    except asyncio.TimeoutError:
        print("[ICE] Warning: timed out — sending partial candidates")


# ======================================
# ROUTES
# ======================================

async def index(request):
    return web.FileResponse("./index.html")


async def offer(request):
    params = await request.json()
    pc = RTCPeerConnection()
    pcs.add(pc)

    print(f"[WebRTC] New peer connection — total: {len(pcs)}")

    # ── THE FIX ────────────────────────────────────────────────────────────
    #
    # ROOT CAUSE (aiortc source, rtcpeerconnection.py line ~251):
    #   self.__stream_id = str(uuid.uuid4())
    #
    # aiortc assigns ONE stream_id to the entire RTCPeerConnection instance
    # and reuses it for every track added via addTrack(). When the browser
    # sends a recvonly offer (no streams), all answer tracks share this same
    # stream_id in their a=msid lines.
    #
    # Chrome, Android Chrome, Edge, Firefox: reject duplicate/shared msid
    # and silently skip firing ontrack → black screen.
    # Safari: lenient, fires ontrack anyway → worked only on iOS.
    #
    # FIX: Before calling addTrack(), overwrite the private __stream_id
    # on the pc instance using Python name mangling notation. This gives
    # every connection a fresh UUID that aiortc will use for the a=msid
    # line in the answer SDP, satisfying all strict browsers.
    #
    # This is safer than string-patching the SDP (which broke iOS last time)
    # because we let aiortc build the SDP normally — we just change the
    # source value it reads.
    #
    unique_stream_id = str(uuid.uuid4())
    pc._RTCPeerConnection__stream_id = unique_stream_id
    print(f"[msid] Assigned unique stream_id: {unique_stream_id[:8]}…")

    @pc.on("connectionstatechange")
    async def on_connectionstatechange():
        print(f"[WebRTC] Connection state: {pc.connectionState}")
        if pc.connectionState in ("failed", "closed"):
            await pc.close()
            pcs.discard(pc)

    track = WebcamTrack(shared_camera)
    pc.addTrack(track)

    await pc.setRemoteDescription(RTCSessionDescription(
        sdp=params["sdp"], type=params["type"]
    ))

    answer = await pc.createAnswer()
    await pc.setLocalDescription(answer)
    await wait_for_ice(pc)

    # Verify the unique msid appears in the answer (visible in server logs)
    msid_lines = [l for l in pc.localDescription.sdp.splitlines() if "msid" in l]
    print(f"[SDP] msid lines: {msid_lines}")

    return web.json_response({
        "sdp": pc.localDescription.sdp,
        "type": pc.localDescription.type
    })


async def on_startup(app):
    shared_camera.start()


async def on_shutdown(app):
    coros = [pc.close() for pc in pcs]
    await asyncio.gather(*coros)
    pcs.clear()
    shared_camera.stop()


# ======================================
# STARTUP
# ======================================

USE_GPU = check_gpu()
shared_camera = SharedCamera()
shared_camera.use_gpu = USE_GPU

app = web.Application()
app.on_startup.append(on_startup)
app.on_shutdown.append(on_shutdown)

app.router.add_get("/", index)
app.router.add_get("/index.html", index)
app.router.add_post("/offer", offer)
app.router.add_static("/", path="./", show_index=False)

web.run_app(app, host="0.0.0.0", port=8080)