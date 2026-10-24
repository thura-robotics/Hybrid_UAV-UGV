let pc = null;
let detectionEnabled = false;

const remoteVideo = document.getElementById("remoteVideo");
const connectButton = document.getElementById("connectButton");
const disconnectButton = document.getElementById("disconnectButton");
const toggleDetectionButton = document.getElementById("toggleDetectionButton");
const snapshotButton = document.getElementById("snapshotButton");
const statusText = document.getElementById("statusText");

disconnectButton.disabled = true;
toggleDetectionButton.disabled = true;
snapshotButton.disabled = true;

connectButton.addEventListener("click", connect);
disconnectButton.addEventListener("click", disconnect);
toggleDetectionButton.addEventListener("click", toggleDetection);
snapshotButton.addEventListener("click", takeSnapshot);


// ======================================
// CONNECT FUNCTION
// ======================================

async function connect() {
    try {
        statusText.innerText = "Status: Connecting...";
        connectButton.disabled = true;

        const configuration = {
            iceServers: [
                { urls: "stun:stun.l.google.com:19302" },
                {
                    urls: "turn:openrelay.metered.ca:80",
                    username: "openrelayproject",
                    credential: "openrelayproject"
                },
                {
                    urls: "turn:openrelay.metered.ca:443",
                    username: "openrelayproject",
                    credential: "openrelayproject"
                }
            ]
        };

        pc = new RTCPeerConnection(configuration);
        pc.addTransceiver("video", { direction: "recvonly" });

        pc.ontrack = (event) => {
            console.log("[ontrack] streams:", event.streams.length, "kind:", event.track.kind);

            // Use streams[0] if present, otherwise wrap the bare track.
            // Both cases are handled because browser behaviour varies.
            remoteVideo.srcObject = (event.streams && event.streams[0])
                ? event.streams[0]
                : new MediaStream([event.track]);

            // Explicit play() is required because:
            // - ontrack is not a user gesture, so autoplay may be blocked
            // - Safari requires play() to be called explicitly
            // - This call is inside the Connect button's async chain,
            //   which IS a user gesture, so all browsers permit it
            remoteVideo.play().catch(err =>
                console.warn("[video] play() prevented:", err.message)
            );
        };

        pc.oniceconnectionstatechange = () => {
            console.log("[ICE] state:", pc.iceConnectionState);
            if (pc.iceConnectionState === "failed" || pc.iceConnectionState === "disconnected") {
                statusText.innerText = "Status: Connection lost";
            }
        };

        const offer = await pc.createOffer();
        await pc.setLocalDescription(offer);

        const response = await fetch("/offer", {
            method: "POST",
            headers: { "Content-Type": "application/json" },
            body: JSON.stringify({
                sdp: pc.localDescription.sdp,
                type: pc.localDescription.type
            })
        });

        if (!response.ok) throw new Error(`Server responded with ${response.status}`);

        const answer = await response.json();
        await pc.setRemoteDescription(answer);

        statusText.innerText = "Status: Connected";
        connectButton.disabled = true;
        disconnectButton.disabled = false;
        toggleDetectionButton.disabled = false;
        snapshotButton.disabled = false;

    } catch (err) {
        console.error("[connect] error:", err);
        statusText.innerText = "Status: Error — " + err.message;
        if (pc) { pc.close(); pc = null; }
        connectButton.disabled = false;
    }
}


// ======================================
// DISCONNECT
// ======================================

function disconnect() {
    if (pc) { pc.close(); pc = null; }
    remoteVideo.srcObject = null;
    detectionEnabled = false;
    toggleDetectionButton.textContent = "Toggle Detection";
    statusText.innerText = "Status: Disconnected";
    connectButton.disabled = false;
    disconnectButton.disabled = true;
    toggleDetectionButton.disabled = true;
    snapshotButton.disabled = true;
}


// ======================================
// TOGGLE DETECTION
// ======================================

function toggleDetection() {
    detectionEnabled = !detectionEnabled;
    toggleDetectionButton.textContent = detectionEnabled ? "Detection: ON" : "Detection: OFF";
    console.log("[Detection] state:", detectionEnabled);
}


// ======================================
// SNAPSHOT
// ======================================

function takeSnapshot() {
    if (!remoteVideo.srcObject) {
        alert("No video stream available.");
        return;
    }
    const canvas = document.createElement("canvas");
    canvas.width = remoteVideo.videoWidth;
    canvas.height = remoteVideo.videoHeight;
    canvas.getContext("2d").drawImage(remoteVideo, 0, 0);
    const link = document.createElement("a");
    link.href = canvas.toDataURL("image/png");
    link.download = "snapshot.png";
    link.click();
}