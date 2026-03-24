const statusEl = document.getElementById("status");
const modeEl = document.getElementById("mode");
const imuEl = document.getElementById("imu");
const imuAckEl = document.getElementById("imuAck");
const debugEl = document.getElementById("debug");
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
let connected = false;
let currentMode = "manual";
let vrArmed = false;
let baseline = { pitch: 0, roll: 0, yaw: 0 };
let latestRaw = { pitch: 0, roll: 0, yaw: 0 };
let renderRaf = 0;
let reconnectTimer = null;
let hasSensorData = false;
let sensorPermissionGranted = false;
let sensorCheckTimer = null;
let orientationEventCount = 0;
let lastDebug = "Debug: -";
let lastUiCommandSeq = {
  sessionStart: 0,
  sessionStop: 0,
  zeroCalibrate: 0,
};

function setStatus(msg) {
  statusEl.textContent = msg;
}

function setDebug(msg) {
  lastDebug = `Debug: ${msg}`;
  debugEl.textContent = lastDebug;
  console.log(`[VRDBG] ${msg}`);
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

async function sendImuSample() {
  const value = recenteredValue(latestRaw);
  await fetch("/imu", {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify({
      type: "imu",
      t: Date.now(),
      pitch: value.pitch,
      roll: value.roll,
      yaw: value.yaw,
    }),
  });
}

async function zeroCalibrate() {
  if (!connected) {
    setStatus("Connect first");
    return;
  }
  if (!hasSensorData) {
    setStatus("Zero calibrate ignored: sensor data unavailable");
    return;
  }
  baseline = { ...latestRaw };
  vrArmed = true;
  setStatus("Zero calibrated. VR pan/tilt active.");
  await setPtzMode("vr");
  try {
    await sendImuSample();
  } catch (_) {
    setStatus("Zero calibrated, but first IMU send failed.");
  }
  await notifySessionState(true, true);
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
  }
}

async function fetchUiCommands() {
  const res = await fetch("/vr/ui-commands");
  if (!res.ok) {
    throw new Error(`UI command fetch failed: ${res.status}`);
  }
  return res.json();
}

async function ensureOrientationPermission() {
  setDebug("Requesting sensor permission");
  if (
    typeof DeviceOrientationEvent !== "undefined" &&
    typeof DeviceOrientationEvent.requestPermission === "function"
  ) {
    const result = await DeviceOrientationEvent.requestPermission();
    if (result !== "granted") {
      setDebug(`Sensor permission denied: ${result}`);
      throw new Error("DeviceOrientation permission denied");
    }
    setDebug(`Sensor permission granted: ${result}`);
  } else {
    setDebug("Sensor permission API not required");
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

function startOrientationFeed() {
  orientationHandler = (evt) => {
    orientationEventCount += 1;
    const hasFiniteAngles =
      Number.isFinite(evt.alpha) || Number.isFinite(evt.beta) || Number.isFinite(evt.gamma);
    if (hasFiniteAngles) {
      hasSensorData = true;
      sensorPermissionGranted = true;
      if (orientationEventCount === 1) {
        setDebug(
          `First deviceorientation alpha=${Number(evt.alpha || 0).toFixed(1)} beta=${Number(
            evt.beta || 0
          ).toFixed(1)} gamma=${Number(evt.gamma || 0).toFixed(1)}`
        );
      }
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
  setDebug("deviceorientation listener attached");

  sendInterval = setInterval(() => {
    if (!connected || !vrArmed || !hasSensorData) {
      return;
    }

    sendImuSample().catch(() => {
      setStatus("IMU send error");
    });
  }, 33);
}

async function disconnect({ releaseMode = true, statusMessage = "Idle" } = {}) {
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
    stopRenderLoop();
    connected = false;
    vrArmed = false;
    renderConnectButton();
    streamSource.src = "";
    if (reconnectTimer) {
      clearTimeout(reconnectTimer);
      reconnectTimer = null;
    }
    imuAckEl.textContent = "Server IMU: -";
    clearCanvases();
    setStatus(statusMessage);
    await notifySessionState(false, false);
    connectBtn.disabled = false;
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

      if (connected && vrArmed && body.mode !== "vr") {
        vrArmed = false;
        await notifySessionState(true, false);
        setStatus("VR input lost. Press 0점 조정 to resume.");
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

function drawCover(ctx, canvas, img) {
  const cw = canvas.width;
  const ch = canvas.height;
  if (!cw || !ch || !img.naturalWidth || !img.naturalHeight) {
    return;
  }

  const scale = Math.max(cw / img.naturalWidth, ch / img.naturalHeight);
  const sw = Math.floor(cw / scale);
  const sh = Math.floor(ch / scale);
  const sx = Math.floor((img.naturalWidth - sw) / 2);
  const sy = Math.floor((img.naturalHeight - sh) / 2);

  ctx.drawImage(img, sx, sy, sw, sh, 0, 0, cw, ch);
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

function bindStreamEvents() {
  streamSource.onload = () => {
    setStatus("Video stream connected");
    startRenderLoop();
  };

  streamSource.onerror = () => {
    setStatus("Video stream error - retrying...");
    if (!connected) {
      return;
    }
    if (reconnectTimer) {
      clearTimeout(reconnectTimer);
    }
    reconnectTimer = setTimeout(() => {
      const token = Date.now();
      streamSource.src = `/stream.mjpg?t=${token}`;
    }, 800);
  };
}

async function connect({ requestPermission = true } = {}) {
  if (connected) {
    await disconnect({
      releaseMode: true,
      statusMessage: "Manual mode active",
    });
    return;
  }

  connectBtn.disabled = true;
  setStatus("Requesting sensor permission...");

  try {
    if (!window.isSecureContext) {
      setStatus("Warning: sensor access may be blocked on insecure HTTP");
    }
    if (requestPermission) {
      await ensureOrientationPermission();
    } else if (!sensorPermissionGranted) {
      throw new Error("Sensor permission requires one phone tap first");
    }

    const health = await fetch("/health");
    if (!health.ok) {
      throw new Error(`Server not ready: ${health.status}`);
    }

    connected = true;
    vrArmed = false;
    renderConnectButton();
    hasSensorData = false;
    orientationEventCount = 0;
    startOrientationFeed();
    startImuAckPolling();
    await notifySessionState(true, false);

    sensorCheckTimer = setTimeout(() => {
      if (connected && !hasSensorData) {
        setDebug(`No IMU events after 3s (count=${orientationEventCount})`);
        setStatus("No IMU events detected. PTZ will fall back to manual mode.");
      }
    }, 3000);

    const token = Date.now();
    streamSource.src = `/stream.mjpg?t=${token}`;
    startRenderLoop();
    setStatus("Video connected. Press 0점 조정 to start VR pan/tilt.");
  } catch (err) {
    await disconnect({
      releaseMode: true,
      statusMessage: `Error: ${err.message}`,
    });
  } finally {
    connectBtn.disabled = false;
  }
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
  } catch (_) {
  }
  renderConnectButton();
}

async function handleUiCommands() {
  try {
    const body = await fetchUiCommands();
    const sessionStartSeq = Number(body.session_start_seq || 0);
    const sessionStopSeq = Number(body.session_stop_seq || 0);
    const zeroCalibrateSeq = Number(body.zero_calibrate_seq || 0);

    if (sessionStartSeq > lastUiCommandSeq.sessionStart) {
      lastUiCommandSeq.sessionStart = sessionStartSeq;
      if (!connected) {
        try {
          await connect({ requestPermission: false });
        } catch (_) {
          setStatus("Controller VR start failed. Tap phone once to grant sensor permission.");
        }
      }
    }

    if (sessionStopSeq > lastUiCommandSeq.sessionStop) {
      lastUiCommandSeq.sessionStop = sessionStopSeq;
      if (connected) {
        await disconnect({
          releaseMode: true,
          statusMessage: "Manual mode active",
        });
      }
    }

    if (zeroCalibrateSeq > lastUiCommandSeq.zeroCalibrate) {
      lastUiCommandSeq.zeroCalibrate = zeroCalibrateSeq;
      if (connected && hasSensorData) {
        await zeroCalibrate();
      } else if (connected) {
        setStatus("Zero calibrate ignored: sensor data unavailable");
      }
    }
  } catch (_) {
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

bindStreamEvents();
initializeMode();
startUiCommandPolling();
window.addEventListener("resize", () => {
  if (connected) {
    resizeCanvasToDisplaySize(leftCanvas);
    resizeCanvasToDisplaySize(rightCanvas);
  }
});
connectBtn.addEventListener("click", () => {
  connect({ requestPermission: true }).catch(() => {
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
  if (connected) {
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
