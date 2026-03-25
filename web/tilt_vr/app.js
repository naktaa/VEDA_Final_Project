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
let latestRaw = { pitch: 0, roll: 0, yaw: 0 };
let latestQuaternion = { x: 0, y: 0, z: 0, w: 1 };
let baselineQuaternion = { x: 0, y: 0, z: 0, w: 1 };
let renderRaf = 0;
let reconnectTimer = null;
let hasSensorData = false;
let sensorCheckTimer = null;
let orientationEventCount = 0;
let imuSendInFlight = false;
let initialSensorWaitResolve = null;
let initialSensorWaitReject = null;
let initialSensorWaitTimer = null;
let pendingInitialZero = false;

const imuSendIntervalMs = 16;
const initialSensorWaitMs = 2000;
const degToRad = Math.PI / 180;
const radToDeg = 180 / Math.PI;

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

function clamp(value, min, max) {
  return Math.max(min, Math.min(max, value));
}

function normalizeQuaternion(q) {
  const norm = Math.hypot(q.x, q.y, q.z, q.w);
  if (!Number.isFinite(norm) || norm < 1e-9) {
    return { x: 0, y: 0, z: 0, w: 1 };
  }

  return {
    x: q.x / norm,
    y: q.y / norm,
    z: q.z / norm,
    w: q.w / norm,
  };
}

function conjugateQuaternion(q) {
  return { x: -q.x, y: -q.y, z: -q.z, w: q.w };
}

function multiplyQuaternion(a, b) {
  return normalizeQuaternion({
    x: a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
    y: a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
    z: a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w,
    w: a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z,
  });
}

function quaternionFromAngles({ pitch, roll, yaw }) {
  const x = (pitch || 0) * degToRad;
  const y = (roll || 0) * degToRad;
  const z = (yaw || 0) * degToRad;

  const cosX = Math.cos(0.5 * x);
  const sinX = Math.sin(0.5 * x);
  const cosY = Math.cos(0.5 * y);
  const sinY = Math.sin(0.5 * y);
  const cosZ = Math.cos(0.5 * z);
  const sinZ = Math.sin(0.5 * z);

  return normalizeQuaternion({
    x: sinX * cosY * cosZ - cosX * sinY * sinZ,
    y: cosX * sinY * cosZ + sinX * cosY * sinZ,
    z: cosX * cosY * sinZ + sinX * sinY * cosZ,
    w: cosX * cosY * cosZ - sinX * sinY * sinZ,
  });
}

function rotationMatrixFromQuaternion(q) {
  const qq = normalizeQuaternion(q);
  const xx = qq.x * qq.x;
  const yy = qq.y * qq.y;
  const zz = qq.z * qq.z;
  const xy = qq.x * qq.y;
  const xz = qq.x * qq.z;
  const yz = qq.y * qq.z;
  const wx = qq.w * qq.x;
  const wy = qq.w * qq.y;
  const wz = qq.w * qq.z;

  return {
    m11: 1 - 2 * (yy + zz),
    m12: 2 * (xy - wz),
    m13: 2 * (xz + wy),
    m21: 2 * (xy + wz),
    m22: 1 - 2 * (xx + zz),
    m23: 2 * (yz - wx),
    m31: 2 * (xz - wy),
    m32: 2 * (yz + wx),
    m33: 1 - 2 * (xx + yy),
  };
}

function zxyAnglesFromQuaternion(q) {
  const m = rotationMatrixFromQuaternion(q);
  const beta = Math.asin(clamp(m.m32, -1, 1));
  const cosBeta = Math.cos(beta);

  let alpha = 0;
  let gamma = 0;
  if (Math.abs(cosBeta) > 1e-6) {
    alpha = Math.atan2(-m.m12, m.m22);
    gamma = Math.atan2(-m.m31, m.m33);
  } else {
    alpha = Math.atan2(m.m21, m.m11);
  }

  return {
    pitch: normalizeAngle(beta * radToDeg),
    roll: normalizeAngle(gamma * radToDeg),
    yaw: normalizeAngle(alpha * radToDeg),
  };
}

function recenteredValue() {
  const relativeQuaternion = multiplyQuaternion(
    conjugateQuaternion(baselineQuaternion),
    latestQuaternion
  );
  return zxyAnglesFromQuaternion(relativeQuaternion);
}

function zeroCalibrate() {
  baselineQuaternion = { ...latestQuaternion };
  setStatus("Zero calibrated");
}

function clearInitialSensorWait() {
  if (initialSensorWaitTimer) {
    clearTimeout(initialSensorWaitTimer);
    initialSensorWaitTimer = null;
  }
  initialSensorWaitResolve = null;
  initialSensorWaitReject = null;
}

function resolveInitialSensorWait() {
  if (!initialSensorWaitResolve) {
    return;
  }
  const resolve = initialSensorWaitResolve;
  clearInitialSensorWait();
  resolve();
}

function rejectInitialSensorWait(message) {
  if (!initialSensorWaitReject) {
    return;
  }
  const reject = initialSensorWaitReject;
  clearInitialSensorWait();
  reject(new Error(message));
}

function waitForInitialSensorData(timeoutMs = initialSensorWaitMs) {
  if (hasSensorData) {
    return Promise.resolve();
  }

  clearInitialSensorWait();
  return new Promise((resolve, reject) => {
    initialSensorWaitResolve = resolve;
    initialSensorWaitReject = reject;
    initialSensorWaitTimer = setTimeout(() => {
      rejectInitialSensorWait("No IMU sensor events detected");
    }, timeoutMs);
  });
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
      const firstSensorEvent = !hasSensorData;
      hasSensorData = true;
      latestQuaternion = quaternionFromAngles(getPitchRollYaw(evt));
      if (pendingInitialZero && firstSensorEvent) {
        pendingInitialZero = false;
        latestRaw = getPitchRollYaw(evt);
        zeroCalibrate();
        resolveInitialSensorWait();
      }
    }

    latestRaw = getPitchRollYaw(evt);
    latestQuaternion = quaternionFromAngles(latestRaw);
    const value = recenteredValue();
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
    if (!connected || currentMode !== "vr" || !hasSensorData || imuSendInFlight) {
      return;
    }

    const value = recenteredValue();
    imuSendInFlight = true;
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
    })
      .catch(() => {
        setStatus("IMU send error");
      })
      .finally(() => {
        imuSendInFlight = false;
      });
  }, imuSendIntervalMs);
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
    clearInitialSensorWait();
    pendingInitialZero = false;
    imuSendInFlight = false;
    stopOrientationFeed();
    stopRenderLoop();
    connected = false;
    renderConnectButton();
    streamSource.src = "";
    if (reconnectTimer) {
      clearTimeout(reconnectTimer);
      reconnectTimer = null;
    }
    imuAckEl.textContent = "Server IMU: -";
    clearCanvases();
    setStatus(statusMessage);
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

async function connect() {
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
    await ensureOrientationPermission();

    hasSensorData = false;
    orientationEventCount = 0;
    pendingInitialZero = true;
    startOrientationFeed();
    setStatus("Waiting for phone IMU...");
    await waitForInitialSensorData();

    const health = await fetch("/health");
    if (!health.ok) {
      throw new Error(`Server not ready: ${health.status}`);
    }

    await setPtzMode("vr");

    connected = true;
    renderConnectButton();
    startImuAckPolling();

    sensorCheckTimer = setTimeout(() => {
      if (connected && !hasSensorData) {
        setStatus("No IMU events detected. PTZ will fall back to manual mode.");
      }
    }, 3000);

    const token = Date.now();
    streamSource.src = `/stream.mjpg?t=${token}`;
    startRenderLoop();
    setStatus("VR mode active. Connecting video stream...");
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

bindStreamEvents();
initializeMode();
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
    fetch("/ptz/mode", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ mode: "manual" }),
      keepalive: true,
    }).catch(() => {});
  }
});
