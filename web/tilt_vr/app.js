const statusEl = document.getElementById("status");
const modeEl = document.getElementById("mode");
const imuEl = document.getElementById("imu");
const imuAckEl = document.getElementById("imuAck");
const debugEl = document.getElementById("debugLog");
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
let uiCommandInterval = null;
let sensorCheckTimer = null;
let renderRaf = 0;
let peerConnection = null;
let lastVideoWaitLogAt = 0;
let reconnectTimer = null;

let sessionActive = false;
let connectInFlight = false;
let currentMode = "manual";
let hasSensorData = false;
let sensorPermissionGranted = false;
let latestRaw = { pitch: 0, roll: 0, yaw: 0 };

let lastUiCommandSeq = {
  sessionStart: 0,
  sessionStop: 0,
  zeroCalibrate: 0,
};

const debugLines = [];

function setStatus(msg) {
  statusEl.textContent = msg;
}

function appendDebug(msg) {
  const line = `${new Date().toLocaleTimeString()} ${msg}`;
  debugLines.push(line);
  while (debugLines.length > 10) {
    debugLines.shift();
  }
  if (debugEl) {
    debugEl.textContent = debugLines.join("\n");
  }
  console.log(`[VRWEB] ${msg}`);
}

function setMode(mode) {
  currentMode = mode === "vr" ? "vr" : "manual";
  modeEl.textContent = `PTZ Mode: ${currentMode}`;
}

function renderConnectButton() {
  connectBtn.textContent = sessionActive ? "Stop Video + VR" : "Connect + Start VR";
}

function getPitchRollYaw(evt) {
  return {
    pitch: Number.isFinite(evt.beta) ? evt.beta : 0,
    roll: Number.isFinite(evt.gamma) ? evt.gamma : 0,
    yaw: Number.isFinite(evt.alpha) ? evt.alpha : 0,
  };
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
  sensorPermissionGranted = true;
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

async function notifySessionState(active, vrModeActive) {
  try {
    await fetch("/vr/session-state", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ active, vr_mode_active: vrModeActive }),
      keepalive: true,
    });
  } catch (_) {
    // State sync failure is non-fatal for local UI flow.
  }
}

async function requestZeroCalibration() {
  const res = await fetch("/ptz/zero", {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify(latestRaw),
  });
  if (!res.ok) {
    const text = await res.text();
    throw new Error(text || `Zero calibration failed: ${res.status}`);
  }
  return res.json();
}

async function zeroCalibrate() {
  if (!hasSensorData) {
    setStatus("Zero calibrate ignored: sensor data unavailable");
    return;
  }

  try {
    await requestZeroCalibration();
    setStatus("Zero calibrated on server");
  } catch (err) {
    setStatus(`Zero calibrate error: ${err.message}`);
  }
}

function startOrientationFeed() {
  if (orientationHandler) {
    return;
  }

  orientationHandler = (evt) => {
    const hasFiniteAngles =
      Number.isFinite(evt.alpha) || Number.isFinite(evt.beta) || Number.isFinite(evt.gamma);
    if (hasFiniteAngles) {
      if (!hasSensorData) {
        appendDebug(
          `First IMU sample alpha=${Number(evt.alpha || 0).toFixed(1)} beta=${Number(
            evt.beta || 0
          ).toFixed(1)} gamma=${Number(evt.gamma || 0).toFixed(1)}`
        );
      }
      hasSensorData = true;
      sensorPermissionGranted = true;
    }

    latestRaw = getPitchRollYaw(evt);
    if (!hasSensorData) {
      imuEl.textContent = "IMU: no sensor data";
      return;
    }

    imuEl.textContent =
      `IMU raw pitch:${latestRaw.pitch.toFixed(1)} ` +
      `roll:${latestRaw.roll.toFixed(1)} yaw:${latestRaw.yaw.toFixed(1)}`;
  };

  window.addEventListener("deviceorientation", orientationHandler, true);

  sendInterval = setInterval(() => {
    if (!sessionActive || currentMode !== "vr" || !hasSensorData) {
      return;
    }

    fetch("/imu", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({
        type: "imu",
        t: Date.now(),
        pitch: latestRaw.pitch,
        roll: latestRaw.roll,
        yaw: latestRaw.yaw,
      }),
    }).catch(() => {
      setStatus("IMU send error");
    });
  }, 33);
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

async function stopWebRtcStream() {
  const oldPc = peerConnection;
  peerConnection = null;
  appendDebug("Stopping WebRTC stream");

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
    // Ignore cleanup errors.
  }
}

function startMjpegStream() {
  if (reconnectTimer) {
    clearTimeout(reconnectTimer);
    reconnectTimer = null;
  }
  const token = Date.now();
  streamSource.src = `/stream.mjpg?t=${token}`;
  appendDebug(`Starting MJPEG stream token=${token}`);
}

function stopMjpegStream() {
  if (reconnectTimer) {
    clearTimeout(reconnectTimer);
    reconnectTimer = null;
  }
  streamSource.removeAttribute("src");
  streamSource.src = "";
  appendDebug("Stopping MJPEG stream");
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

function getSourceWidth(mediaEl) {
  return mediaEl.naturalWidth || mediaEl.videoWidth || 0;
}

function getSourceHeight(mediaEl) {
  return mediaEl.naturalHeight || mediaEl.videoHeight || 0;
}

function drawCover(ctx, canvas, mediaEl) {
  const cw = canvas.width;
  const ch = canvas.height;
  const sourceWidth = getSourceWidth(mediaEl);
  const sourceHeight = getSourceHeight(mediaEl);
  if (!cw || !ch || !sourceWidth || !sourceHeight) {
    return;
  }

  const scale = Math.max(cw / sourceWidth, ch / sourceHeight);
  const sw = Math.floor(cw / scale);
  const sh = Math.floor(ch / scale);
  const sx = Math.floor((sourceWidth - sw) / 2);
  const sy = Math.floor((sourceHeight - sh) / 2);

  ctx.drawImage(mediaEl, sx, sy, sw, sh, 0, 0, cw, ch);
}

function renderStereo() {
  if (!sessionActive) {
    renderRaf = 0;
    return;
  }

  if (!getSourceWidth(streamSource) || !getSourceHeight(streamSource)) {
    const now = Date.now();
    if (now - lastVideoWaitLogAt > 1000) {
      appendDebug("Waiting for decoded video frames");
      lastVideoWaitLogAt = now;
    }
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
  appendDebug(`Sending WebRTC offer (${offer.sdp.length} bytes)`);
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

  appendDebug("Received WebRTC answer");
  return res.json();
}

async function startWebRtcStream() {
  if (peerConnection) {
    return;
  }

  const pc = new RTCPeerConnection({
    sdpSemantics: "unified-plan",
    iceServers: [],
  });
  peerConnection = pc;
  appendDebug("Created RTCPeerConnection");

  pc.addTransceiver("video", { direction: "recvonly" });
  appendDebug("Added recvonly video transceiver");

  pc.ontrack = async (event) => {
    const [stream] = event.streams;
    appendDebug(
      `ontrack kind=${event.track?.kind ?? "unknown"} streams=${event.streams.length}`
    );
    if (!stream) {
      return;
    }
    const [videoTrack] = stream.getVideoTracks();
    if (videoTrack) {
      appendDebug(`Video track id=${videoTrack.id} readyState=${videoTrack.readyState}`);
      videoTrack.onmute = () => appendDebug("Video track muted");
      videoTrack.onunmute = () => appendDebug("Video track unmuted");
      videoTrack.onended = () => appendDebug("Video track ended");
    }
    streamSource.srcObject = stream;
    try {
      await streamSource.play();
      appendDebug("Hidden video element play() resolved");
    } catch (_) {
      appendDebug("Hidden video element play() deferred");
    }
    startRenderLoop();
    setStatus(currentMode === "vr" ? "WebRTC video connected" : "Video connected");
  };

  pc.onconnectionstatechange = () => {
    appendDebug(`Peer connection state=${pc.connectionState}`);
    if (!sessionActive) {
      return;
    }
    if (pc.connectionState === "failed") {
      setStatus("WebRTC connection failed");
    } else if (pc.connectionState === "disconnected") {
      setStatus("WebRTC disconnected");
    }
  };

  pc.oniceconnectionstatechange = () => {
    appendDebug(`ICE connection state=${pc.iceConnectionState}`);
    if (!sessionActive) {
      return;
    }
    if (pc.iceConnectionState === "failed") {
      setStatus("ICE failed");
    } else if (pc.iceConnectionState === "disconnected") {
      setStatus("ICE disconnected");
    }
  };

  const offer = await pc.createOffer({
    offerToReceiveAudio: false,
    offerToReceiveVideo: true,
  });
  appendDebug("Created local WebRTC offer");
  await pc.setLocalDescription(offer);
  await waitForIceGatheringComplete(pc);
  appendDebug("ICE gathering completed in browser");

  const answer = await requestWebRtcAnswer(pc.localDescription);
  await pc.setRemoteDescription({
    type: answer.type,
    sdp: answer.sdp,
  });
  appendDebug("Applied remote WebRTC answer");
}

function startImuAckPolling() {
  if (imuAckInterval) {
    return;
  }

  imuAckInterval = setInterval(async () => {
    if (!sessionActive) {
      return;
    }

    try {
      const res = await fetch("/imu/latest");
      if (!res.ok) {
        return;
      }
      const body = await res.json();
      const previousMode = currentMode;
      setMode(body.mode);
      await notifySessionState(sessionActive, body.mode === "vr");
      imuAckEl.textContent =
        `Server mode:${body.mode} pitch:${Number(body.pitch).toFixed(1)} ` +
        `roll:${Number(body.roll).toFixed(1)} ` +
        `yaw:${Number(body.yaw).toFixed(1)} ` +
        `t:${body.t} servo:${body.servo_ready ? "ready" : "off"} ` +
        `src:${body.source ?? "-"}`;

      if (previousMode === "vr" && body.mode !== "vr") {
        appendDebug("PTZ mode fell back to manual");
        setStatus("VR input lost. Video continues in manual mode.");
      }
    } catch (_) {
      imuAckEl.textContent = "Server IMU: polling error";
    }
  }, 500);
}

async function stopSession(statusMessage = "Idle") {
  connectInFlight = true;
  connectBtn.disabled = true;
  appendDebug(`Stopping session: ${statusMessage}`);

  try {
    try {
      await setPtzMode("manual");
    } catch (_) {
      setMode("manual");
    }

    stopOrientationFeed();
    stopMjpegStream();
    stopRenderLoop();
    sessionActive = false;
    renderConnectButton();
    imuAckEl.textContent = "Server IMU: -";
    clearCanvases();
    setStatus(statusMessage);
  } finally {
    await notifySessionState(false, false);
    connectBtn.disabled = false;
    connectInFlight = false;
  }
}

async function startSession({ requestPermission = true } = {}) {
  if (connectInFlight) {
    return;
  }

  connectInFlight = true;
  connectBtn.disabled = true;
  appendDebug(`Starting session requestPermission=${requestPermission}`);

  try {
    if (requestPermission) {
      if (!window.isSecureContext) {
        setStatus("Warning: sensor access may be blocked on insecure HTTP");
      }
      await ensureOrientationPermission();
    } else if (!sensorPermissionGranted) {
      throw new Error("Sensor permission requires one phone tap first");
    }

    const health = await fetch("/health");
    if (!health.ok) {
      throw new Error(`Server not ready: ${health.status}`);
    }
    appendDebug("Server health check passed");

    startOrientationFeed();
    startImuAckPolling();

    if (!sessionActive) {
      setStatus("Connecting MJPEG stream...");
      startMjpegStream();
      sessionActive = true;
      renderConnectButton();
      await notifySessionState(true, false);
      appendDebug("MJPEG session marked active");
      startRenderLoop();
    }

    await setPtzMode("vr");
    await notifySessionState(true, true);
    appendDebug("PTZ mode switched to vr");
    setStatus("VR mode active");

    if (sensorCheckTimer) {
      clearTimeout(sensorCheckTimer);
    }
    sensorCheckTimer = setTimeout(() => {
      if (sessionActive && !hasSensorData) {
        setStatus("No IMU events detected. Check phone sensor permission.");
      }
    }, 3000);
  } catch (err) {
    if (!sessionActive) {
      await notifySessionState(false, false);
    }
    appendDebug(`Session start error: ${err.message}`);
    setStatus(`Error: ${err.message}`);
    throw err;
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
    const sessionStartSeq = Number(body.session_start_seq || 0);
    const sessionStopSeq = Number(body.session_stop_seq || 0);
    const zeroCalibrateSeq = Number(body.zero_calibrate_seq || 0);

    if (sessionStartSeq > lastUiCommandSeq.sessionStart) {
      lastUiCommandSeq.sessionStart = sessionStartSeq;
      appendDebug(`Controller session start command seq=${sessionStartSeq}`);
      try {
        await startSession({ requestPermission: false });
      } catch (_) {
        setStatus("Controller VR start failed. Tap phone once to grant sensor permission.");
      }
    }

    if (sessionStopSeq > lastUiCommandSeq.sessionStop) {
      lastUiCommandSeq.sessionStop = sessionStopSeq;
      appendDebug(`Controller session stop command seq=${sessionStopSeq}`);
      if (sessionActive) {
        await stopSession("Manual mode active");
      }
    }

    if (zeroCalibrateSeq > lastUiCommandSeq.zeroCalibrate) {
      lastUiCommandSeq.zeroCalibrate = zeroCalibrateSeq;
      appendDebug(`Controller zero command seq=${zeroCalibrateSeq}`);
      if (sessionActive && hasSensorData) {
        await zeroCalibrate();
      } else {
        setStatus("Zero calibrate ignored: session or sensor unavailable");
      }
    }
  } catch (_) {
    if (!sessionActive) {
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

function bindStreamDebugEvents() {
  streamSource.addEventListener("load", () => {
    appendDebug(`mjpeg load ${getSourceWidth(streamSource)}x${getSourceHeight(streamSource)}`);
    if (sessionActive) {
      setStatus(currentMode === "vr" ? "MJPEG video connected" : "Video connected");
    }
  });
  streamSource.addEventListener("error", () => {
    appendDebug("mjpeg element error");
    if (!sessionActive) {
      return;
    }
    setStatus("MJPEG stream error - retrying...");
    if (reconnectTimer) {
      clearTimeout(reconnectTimer);
    }
    reconnectTimer = setTimeout(() => {
      if (sessionActive) {
        startMjpegStream();
      }
    }, 800);
  });
}

async function initializeMode() {
  try {
    await fetchPtzMode();
  } catch (_) {
    setMode("manual");
  }

  try {
    const body = await fetchUiCommands();
    lastUiCommandSeq.sessionStart = Number(body.session_start_seq || 0);
    lastUiCommandSeq.sessionStop = Number(body.session_stop_seq || 0);
    lastUiCommandSeq.zeroCalibrate = Number(body.zero_calibrate_seq || 0);
    sessionActive = Boolean(body.session_active);
    await notifySessionState(sessionActive, currentMode === "vr");
  } catch (_) {
    sessionActive = false;
  }

  renderConnectButton();
}

async function toggleSessionFromButton() {
  if (sessionActive) {
    await stopSession("Manual mode active");
    return;
  }

  try {
    await startSession({ requestPermission: true });
  } catch (_) {
    // Status is already updated in startSession.
  }
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

bindStreamDebugEvents();
initializeMode();
startUiCommandPolling();

window.addEventListener("resize", () => {
  if (sessionActive) {
    resizeCanvasToDisplaySize(leftCanvas);
    resizeCanvasToDisplaySize(rightCanvas);
  }
});

connectBtn.addEventListener("click", () => {
  toggleSessionFromButton().catch(() => {
    setStatus("Connect error");
  });
});

centerBtn.addEventListener("click", () => {
  zeroCalibrate().catch(() => {
    setStatus("Zero calibrate error");
  });
});

fullscreenBtn.addEventListener("click", toggleFullscreen);
scrollBtn.addEventListener("click", scrollToVideo);
document.addEventListener("fullscreenchange", updateFullscreenButton);

window.addEventListener("beforeunload", () => {
  if (sessionActive) {
    fetch("/vr/session-state", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ active: false, vr_mode_active: false }),
      keepalive: true,
    }).catch(() => {});
    fetch("/ptz/mode", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ mode: "manual" }),
      keepalive: true,
    }).catch(() => {});
  }
});
