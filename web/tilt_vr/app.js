const statusEl = document.getElementById("status");
const modeEl = document.getElementById("mode");
const imuEl = document.getElementById("imu");
const imuAckEl = document.getElementById("imuAck");
const connectBtn = document.getElementById("connectBtn");
const centerBtn = document.getElementById("centerBtn");
const fullscreenBtn = document.getElementById("fullscreenBtn");
const scrollBtn = document.getElementById("scrollBtn");
const vrStage = document.querySelector(".vr-stage");

const streamSource = document.getElementById("streamSource");
const leftCanvas = document.getElementById("leftCanvas");
const rightCanvas = document.getElementById("rightCanvas");
const leftCtx = leftCanvas.getContext("2d", { alpha: false });
const rightCtx = rightCanvas.getContext("2d", { alpha: false });

let orientationHandler = null;
let sendInterval = null;
let imuAckInterval = null;
let connected = false;
let currentMode = "manual";
let baseline = { pitch: 0, roll: 0, yaw: 0 };
let latestRaw = { pitch: 0, roll: 0, yaw: 0 };
let renderRaf = 0;
let hasSensorData = false;
let sensorCheckTimer = null;
let orientationEventCount = 0;
let peerConnection = null;
let connectInFlight = false;
let uiCommandInterval = null;
let lastUiCommandSeq = {
  sessionToggle: 0,
  zeroCalibrate: 0,
};

function setStatus(msg) {
  statusEl.textContent = msg;
}

function setMode(mode) {
  currentMode = mode === "vr" ? "vr" : "manual";
  modeEl.textContent = `PTZ Mode: ${currentMode}`;
}

function renderConnectButton() {
  connectBtn.textContent = connected ? "Stop VR + Manual" : "Connect + Start VR";
}

function getPitchRollYaw(evt) {
  return {
    pitch: Number.isFinite(evt.beta) ? evt.beta : 0,
    roll: Number.isFinite(evt.gamma) ? evt.gamma : 0,
    yaw: Number.isFinite(evt.alpha) ? evt.alpha : 0,
  };
}

function normalizeAngle(value) {
  let v = value;
  while (v > 180) v -= 360;
  while (v < -180) v += 360;
  return v;
}

function recenteredValue(raw) {
  return {
    pitch: normalizeAngle(raw.pitch - baseline.pitch),
    roll: normalizeAngle(raw.roll - baseline.roll),
    yaw: normalizeAngle(raw.yaw - baseline.yaw),
  };
}

function zeroCalibrate() {
  baseline = { ...latestRaw };
  setStatus("Zero calibrated");
}

async function ensureOrientationPermission() {
  if (
    typeof DeviceOrientationEvent !== "undefined" &&
    typeof DeviceOrientationEvent.requestPermission === "function"
  ) {
    const result = await DeviceOrientationEvent.requestPermission();
    if (result !== "granted") {
      throw new Error("DeviceOrientation permission denied");
    }
  }
}

async function fetchPtzMode() {
  const res = await fetch("/ptz/mode");
  if (!res.ok) {
    throw new Error(`Mode fetch failed: ${res.status}`);
  }
  const body = await res.json();
  setMode(body.mode);
  return body;
}

async function setPtzMode(mode) {
  const res = await fetch("/ptz/mode", {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify({ mode }),
  });
  if (!res.ok) {
    const errorText = await res.text();
    throw new Error(errorText || `Mode switch failed: ${res.status}`);
  }
  const body = await res.json();
  setMode(body.mode);
  return body;
}

function startOrientationFeed() {
  orientationHandler = (evt) => {
    orientationEventCount += 1;
    const hasFiniteAngles =
      Number.isFinite(evt.alpha) || Number.isFinite(evt.beta) || Number.isFinite(evt.gamma);
    if (hasFiniteAngles) {
      hasSensorData = true;
    }

    latestRaw = getPitchRollYaw(evt);
    const value = recenteredValue(latestRaw);
    if (!hasSensorData) {
      imuEl.textContent = "IMU: no sensor data";
      return;
    }

    imuEl.textContent =
      `IMU pitch:${value.pitch.toFixed(1)} ` +
      `roll:${value.roll.toFixed(1)} yaw:${value.yaw.toFixed(1)}`;
  };

  window.addEventListener("deviceorientation", orientationHandler, true);

  sendInterval = setInterval(() => {
    if (!connected || currentMode !== "vr" || !hasSensorData) {
      return;
    }

    const value = recenteredValue(latestRaw);
    fetch("/imu", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({
        type: "imu",
        t: Date.now(),
        pitch: value.pitch,
        roll: value.roll,
        yaw: value.yaw,
      }),
    }).catch(() => {
      setStatus("IMU send error");
    });
  }, 33);
}

async function stopWebRtcStream() {
  const oldPc = peerConnection;
  peerConnection = null;

  if (oldPc) {
    oldPc.ontrack = null;
    oldPc.onconnectionstatechange = null;
    oldPc.oniceconnectionstatechange = null;
    oldPc.close();
  }

  streamSource.pause();
  streamSource.srcObject = null;

  try {
    await fetch("/webrtc/stop", { method: "POST", keepalive: true });
  } catch (_) {
    // Ignore cleanup errors on shutdown paths.
  }
}

async function disconnect({ releaseMode = true, statusMessage = "Idle" } = {}) {
  connectInFlight = true;
  connectBtn.disabled = true;

  try {
    if (releaseMode) {
      await setPtzMode("manual");
    } else {
      setMode("manual");
    }
  } catch (_) {
    setMode("manual");
  } finally {
    stopOrientationFeed();
    await stopWebRtcStream();
    stopRenderLoop();
    connected = false;
    renderConnectButton();
    imuAckEl.textContent = "Server IMU: -";
    clearCanvases();
    setStatus(statusMessage);
    connectBtn.disabled = false;
    connectInFlight = false;
  }
}

function startImuAckPolling() {
  imuAckInterval = setInterval(async () => {
    if (!connected) {
      return;
    }

    try {
      const res = await fetch("/imu/latest");
      if (!res.ok) {
        return;
      }
      const body = await res.json();
      setMode(body.mode);
      imuAckEl.textContent =
        `Server mode:${body.mode} pitch:${Number(body.pitch).toFixed(1)} ` +
        `roll:${Number(body.roll).toFixed(1)} ` +
        `yaw:${Number(body.yaw).toFixed(1)} ` +
        `t:${body.t} servo:${body.servo_ready ? "ready" : "off"} ` +
        `src:${body.source ?? "-"}`;

      if (connected && body.mode !== "vr") {
        await disconnect({
          releaseMode: false,
          statusMessage: "VR input lost. Back to manual mode.",
        });
      }
    } catch (_) {
      imuAckEl.textContent = "Server IMU: polling error";
    }
  }, 500);
}

function stopOrientationFeed() {
  if (orientationHandler) {
    window.removeEventListener("deviceorientation", orientationHandler, true);
    orientationHandler = null;
  }
  if (sendInterval) {
    clearInterval(sendInterval);
    sendInterval = null;
  }
  if (imuAckInterval) {
    clearInterval(imuAckInterval);
    imuAckInterval = null;
  }
  if (sensorCheckTimer) {
    clearTimeout(sensorCheckTimer);
    sensorCheckTimer = null;
  }
}

function resizeCanvasToDisplaySize(canvas) {
  const dpr = Math.max(1, Math.min(2, window.devicePixelRatio || 1));
  const w = Math.max(1, Math.floor(canvas.clientWidth * dpr));
  const h = Math.max(1, Math.floor(canvas.clientHeight * dpr));
  if (canvas.width !== w || canvas.height !== h) {
    canvas.width = w;
    canvas.height = h;
  }
}

function drawCover(ctx, canvas, video) {
  const cw = canvas.width;
  const ch = canvas.height;
  if (!cw || !ch || !video.videoWidth || !video.videoHeight) {
    return;
  }

  const scale = Math.max(cw / video.videoWidth, ch / video.videoHeight);
  const sw = Math.floor(cw / scale);
  const sh = Math.floor(ch / scale);
  const sx = Math.floor((video.videoWidth - sw) / 2);
  const sy = Math.floor((video.videoHeight - sh) / 2);

  ctx.drawImage(video, sx, sy, sw, sh, 0, 0, cw, ch);
}

function renderStereo() {
  if (!connected) {
    renderRaf = 0;
    return;
  }

  resizeCanvasToDisplaySize(leftCanvas);
  resizeCanvasToDisplaySize(rightCanvas);
  drawCover(leftCtx, leftCanvas, streamSource);
  drawCover(rightCtx, rightCanvas, streamSource);

  renderRaf = requestAnimationFrame(renderStereo);
}

function startRenderLoop() {
  if (!renderRaf) {
    renderRaf = requestAnimationFrame(renderStereo);
  }
}

function stopRenderLoop() {
  if (renderRaf) {
    cancelAnimationFrame(renderRaf);
    renderRaf = 0;
  }
}

function clearCanvases() {
  leftCtx.fillStyle = "#000";
  rightCtx.fillStyle = "#000";
  leftCtx.fillRect(0, 0, leftCanvas.width || 1, leftCanvas.height || 1);
  rightCtx.fillRect(0, 0, rightCanvas.width || 1, rightCanvas.height || 1);
}

function waitForIceGatheringComplete(pc) {
  if (pc.iceGatheringState === "complete") {
    return Promise.resolve();
  }

  return new Promise((resolve) => {
    const onStateChange = () => {
      if (pc.iceGatheringState === "complete") {
        pc.removeEventListener("icegatheringstatechange", onStateChange);
        resolve();
      }
    };
    pc.addEventListener("icegatheringstatechange", onStateChange);
  });
}

async function requestWebRtcAnswer(offer) {
  const res = await fetch("/webrtc/session", {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify({
      type: offer.type,
      sdp: offer.sdp,
    }),
  });

  if (!res.ok) {
    const text = await res.text();
    throw new Error(text || `WebRTC session failed: ${res.status}`);
  }

  return res.json();
}

async function startWebRtcStream() {
  if (peerConnection) {
    await stopWebRtcStream();
  }

  const pc = new RTCPeerConnection({
    sdpSemantics: "unified-plan",
    iceServers: [],
  });
  peerConnection = pc;

  pc.addTransceiver("video", { direction: "recvonly" });

  pc.ontrack = async (event) => {
    const [stream] = event.streams;
    if (!stream) {
      return;
    }
    streamSource.srcObject = stream;
    try {
      await streamSource.play();
    } catch (_) {
      // Ignore autoplay race; the element is muted and usually succeeds.
    }
    startRenderLoop();
    setStatus("WebRTC video connected");
  };

  pc.onconnectionstatechange = () => {
    if (!connected) {
      return;
    }

    if (pc.connectionState === "failed" || pc.connectionState === "disconnected") {
      setStatus(`WebRTC ${pc.connectionState}`);
    }
  };

  pc.oniceconnectionstatechange = () => {
    if (!connected) {
      return;
    }

    if (pc.iceConnectionState === "failed" || pc.iceConnectionState === "disconnected") {
      setStatus(`ICE ${pc.iceConnectionState}`);
    }
  };

  const offer = await pc.createOffer({
    offerToReceiveAudio: false,
    offerToReceiveVideo: true,
  });
  await pc.setLocalDescription(offer);
  await waitForIceGatheringComplete(pc);

  const answer = await requestWebRtcAnswer(pc.localDescription);
  await pc.setRemoteDescription({
    type: answer.type,
    sdp: answer.sdp,
  });
}

async function connect() {
  if (connectInFlight) {
    return;
  }

  if (connected) {
    await disconnect({
      releaseMode: true,
      statusMessage: "Manual mode active",
    });
    return;
  }

  connectInFlight = true;
  connectBtn.disabled = true;
  setStatus("Requesting sensor permission...");

  try {
    if (!window.isSecureContext) {
      setStatus("Warning: sensor access may be blocked on insecure HTTP");
    }
    await ensureOrientationPermission();

    const health = await fetch("/health");
    if (!health.ok) {
      throw new Error(`Server not ready: ${health.status}`);
    }

    await setPtzMode("vr");

    connected = true;
    renderConnectButton();
    hasSensorData = false;
    orientationEventCount = 0;
    zeroCalibrate();
    startOrientationFeed();
    startImuAckPolling();

    sensorCheckTimer = setTimeout(() => {
      if (connected && !hasSensorData) {
        setStatus("No IMU events detected. PTZ will fall back to manual mode.");
      }
    }, 3000);

    setStatus("VR mode active. Negotiating WebRTC...");
    await startWebRtcStream();
  } catch (err) {
    await disconnect({
      releaseMode: true,
      statusMessage: `Error: ${err.message}`,
    });
  } finally {
    connectBtn.disabled = false;
    connectInFlight = false;
  }
}

async function fetchUiCommands() {
  const res = await fetch("/vr/ui-commands");
  if (!res.ok) {
    throw new Error(`UI command fetch failed: ${res.status}`);
  }
  return res.json();
}

async function handleUiCommands() {
  if (connectInFlight) {
    return;
  }

  try {
    const body = await fetchUiCommands();
    const sessionToggleSeq = Number(body.session_toggle_seq || 0);
    const zeroCalibrateSeq = Number(body.zero_calibrate_seq || 0);

    if (sessionToggleSeq > lastUiCommandSeq.sessionToggle) {
      lastUiCommandSeq.sessionToggle = sessionToggleSeq;
      try {
        await connect();
      } catch (_) {
        setStatus("Controller VR toggle failed");
      }
    }

    if (zeroCalibrateSeq > lastUiCommandSeq.zeroCalibrate) {
      lastUiCommandSeq.zeroCalibrate = zeroCalibrateSeq;
      if (connected) {
        zeroCalibrate();
      } else {
        setStatus("Zero calibrate ignored: VR not connected");
      }
    }
  } catch (_) {
    if (!connected) {
      return;
    }
    setStatus("UI command polling error");
  }
}

function startUiCommandPolling() {
  if (uiCommandInterval) {
    clearInterval(uiCommandInterval);
  }

  uiCommandInterval = setInterval(() => {
    handleUiCommands().catch(() => {});
  }, 250);
}

async function initializeMode() {
  try {
    await fetchPtzMode();
  } catch (_) {
    setMode("manual");
  }

  try {
    const body = await fetchUiCommands();
    lastUiCommandSeq.sessionToggle = Number(body.session_toggle_seq || 0);
    lastUiCommandSeq.zeroCalibrate = Number(body.zero_calibrate_seq || 0);
  } catch (_) {
    // Ignore initial command sync failures; polling will retry.
  }

  renderConnectButton();
}

async function toggleFullscreen() {
  try {
    if (!document.fullscreenElement) {
      if (vrStage.requestFullscreen) {
        await vrStage.requestFullscreen();
      } else if (document.documentElement.requestFullscreen) {
        await document.documentElement.requestFullscreen();
      } else {
        setStatus("Fullscreen not supported");
        return;
      }
    } else {
      await document.exitFullscreen();
    }
  } catch (_) {
    setStatus("Fullscreen error");
  }
}

function updateFullscreenButton() {
  fullscreenBtn.textContent = document.fullscreenElement ? "Exit Fullscreen" : "Fullscreen";
}

function scrollToVideo() {
  vrStage.scrollIntoView({ behavior: "smooth", block: "start" });
}

initializeMode();
startUiCommandPolling();
window.addEventListener("resize", () => {
  if (connected) {
    resizeCanvasToDisplaySize(leftCanvas);
    resizeCanvasToDisplaySize(rightCanvas);
  }
});
connectBtn.addEventListener("click", () => {
  connect().catch(() => {
    setStatus("Connect error");
  });
});
centerBtn.addEventListener("click", zeroCalibrate);
fullscreenBtn.addEventListener("click", toggleFullscreen);
scrollBtn.addEventListener("click", scrollToVideo);
document.addEventListener("fullscreenchange", updateFullscreenButton);
window.addEventListener("beforeunload", () => {
  if (connected) {
    fetch("/webrtc/stop", { method: "POST", keepalive: true }).catch(() => {});
    fetch("/ptz/mode", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ mode: "manual" }),
      keepalive: true,
    }).catch(() => {});
  }
});
