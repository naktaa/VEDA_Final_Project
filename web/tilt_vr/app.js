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
const minimapCanvas = document.getElementById("minimapCanvas");
const leftCtx = leftCanvas.getContext("2d", { alpha: false });
const rightCtx = rightCanvas.getContext("2d", { alpha: false });
const minimapCtx = minimapCanvas.getContext("2d");

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
let pendingInitialZero = false;
let vrModeRecoveryInFlight = false;
let overlayStateInterval = null;
let minimapConfig = null;
let latestOverlayState = {
  rc: {
    valid: false,
    stale: false,
    x: 0,
    y: 0,
    yaw_rad: 0,
    frame: "world",
    ts_ms: 0,
  },
};

const imuSendIntervalMs = 16;
const overlayStatePollMs = 100;
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

function waitingForImuStatus(delayed = false) {
  if (!window.isSecureContext) {
    return delayed
      ? "Waiting for phone IMU... This HTTP page may block motion sensors. Open the HTTPS tiltVR address."
      : "Waiting for phone IMU... HTTP may block motion sensors. HTTPS is recommended.";
  }
  return delayed ? "Still waiting for phone IMU..." : "Waiting for phone IMU...";
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

async function requestVrModeRecovery() {
  if (!connected || vrModeRecoveryInFlight || !hasSensorData) {
    return;
  }

  vrModeRecoveryInFlight = true;
  try {
    await setPtzMode("vr");
    setStatus("VR mode recovered");
  } catch (_) {
    setStatus("VR mode recovery retrying...");
  } finally {
    vrModeRecoveryInFlight = false;
  }
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
        if (connected && currentMode !== "vr") {
          requestVrModeRecovery().catch(() => {});
        }
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
    pendingInitialZero = false;
    imuSendInFlight = false;
    vrModeRecoveryInFlight = false;
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
    imuEl.textContent = "IMU: -";
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
        setStatus("VR input unstable. Re-entering VR mode...");
        requestVrModeRecovery().catch(() => {});
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

async function loadMinimapConfig() {
  if (minimapConfig) {
    return minimapConfig;
  }

  const res = await fetch("/web/minimap-config.json", { cache: "no-store" });
  if (!res.ok) {
    throw new Error(`Minimap config failed: ${res.status}`);
  }

  minimapConfig = await res.json();
  drawMinimap();
  return minimapConfig;
}

function sortMarkerEntries(markers) {
  return Object.entries(markers || {}).sort((a, b) => Number(a[0]) - Number(b[0]));
}

function mapPointToCanvas(x, y, bounds, canvas) {
  const pad = 18;
  const spanX = Math.max(1e-6, Number(bounds.maxX) - Number(bounds.minX));
  const spanY = Math.max(1e-6, Number(bounds.maxY) - Number(bounds.minY));
  const innerW = Math.max(1, canvas.width - pad * 2);
  const innerH = Math.max(1, canvas.height - pad * 2);
  const scale = Math.min(innerW / spanX, innerH / spanY);
  const drawW = spanX * scale;
  const drawH = spanY * scale;
  const offsetX = (canvas.width - drawW) * 0.5;
  const offsetY = (canvas.height - drawH) * 0.5;

  return {
    x: offsetX + (x - bounds.minX) * scale,
    y: canvas.height - (offsetY + (y - bounds.minY) * scale),
    scale,
  };
}

function drawPolygon(ctx, canvas, bounds, points, { fillStyle = null, strokeStyle = null, lineWidth = 2 } = {}) {
  if (!Array.isArray(points) || points.length < 2) {
    return;
  }

  ctx.beginPath();
  points.forEach(([x, y], index) => {
    const pt = mapPointToCanvas(x, y, bounds, canvas);
    if (index === 0) {
      ctx.moveTo(pt.x, pt.y);
    } else {
      ctx.lineTo(pt.x, pt.y);
    }
  });
  ctx.closePath();

  if (fillStyle) {
    ctx.fillStyle = fillStyle;
    ctx.fill();
  }
  if (strokeStyle) {
    ctx.lineWidth = lineWidth;
    ctx.strokeStyle = strokeStyle;
    ctx.stroke();
  }
}

function drawMarkerReference(ctx, canvas, bounds, markers) {
  const entries = sortMarkerEntries(markers);
  const orderedIds = ["10", "11", "13", "12"];
  const points = orderedIds
    .map((id) => markers?.[id])
    .filter((value) => Array.isArray(value) && value.length === 2);
  drawPolygon(ctx, canvas, bounds, points, {
    strokeStyle: "rgba(64, 224, 255, 0.95)",
    lineWidth: 2,
  });

  ctx.save();
  ctx.fillStyle = "#f3fbff";
  ctx.font = `${Math.max(12, Math.floor(canvas.width * 0.055))}px sans-serif`;
  ctx.textAlign = "center";
  ctx.textBaseline = "bottom";

  for (const [id, value] of entries) {
    const pt = mapPointToCanvas(value[0], value[1], bounds, canvas);
    ctx.beginPath();
    ctx.fillStyle = "rgba(64, 224, 255, 0.95)";
    ctx.arc(pt.x, pt.y, Math.max(3, canvas.width * 0.018), 0, Math.PI * 2);
    ctx.fill();
    ctx.fillStyle = "#f3fbff";
    ctx.fillText(id, pt.x, pt.y - 7);
  }

  ctx.restore();
}

function drawRcPose(ctx, canvas, bounds, rc) {
  if (!rc || !rc.valid) {
    return;
  }

  const pt = mapPointToCanvas(rc.x, rc.y, bounds, canvas);
  const heading = Number.isFinite(rc.yaw_rad) ? rc.yaw_rad : 0;
  const color = rc.stale ? "rgba(173, 184, 194, 0.78)" : "#ffffff";
  const glow = rc.stale ? "rgba(173, 184, 194, 0.24)" : "rgba(83, 208, 255, 0.34)";
  const radius = Math.max(4, canvas.width * 0.022);
  const arrowLen = Math.max(12, canvas.width * 0.09);
  const tipX = pt.x + Math.cos(heading) * arrowLen;
  const tipY = pt.y - Math.sin(heading) * arrowLen;

  ctx.save();
  ctx.strokeStyle = glow;
  ctx.lineWidth = radius * 3;
  ctx.beginPath();
  ctx.moveTo(pt.x, pt.y);
  ctx.lineTo(tipX, tipY);
  ctx.stroke();

  ctx.strokeStyle = color;
  ctx.lineWidth = 2.5;
  ctx.beginPath();
  ctx.moveTo(pt.x, pt.y);
  ctx.lineTo(tipX, tipY);
  ctx.stroke();

  const sideAngle = Math.PI / 7;
  const headLen = Math.max(7, canvas.width * 0.04);
  ctx.beginPath();
  ctx.moveTo(tipX, tipY);
  ctx.lineTo(
    tipX - Math.cos(heading - sideAngle) * headLen,
    tipY + Math.sin(heading - sideAngle) * headLen
  );
  ctx.lineTo(
    tipX - Math.cos(heading + sideAngle) * headLen,
    tipY + Math.sin(heading + sideAngle) * headLen
  );
  ctx.closePath();
  ctx.fillStyle = color;
  ctx.fill();

  ctx.fillStyle = color;
  ctx.beginPath();
  ctx.arc(pt.x, pt.y, radius, 0, Math.PI * 2);
  ctx.fill();
  ctx.restore();
}

function drawMinimap() {
  resizeCanvasToDisplaySize(minimapCanvas);

  const ctx = minimapCtx;
  const canvas = minimapCanvas;
  ctx.clearRect(0, 0, canvas.width, canvas.height);

  ctx.fillStyle = "rgba(8, 20, 28, 0.42)";
  ctx.fillRect(0, 0, canvas.width, canvas.height);

  if (!minimapConfig) {
    ctx.fillStyle = "rgba(232, 243, 251, 0.72)";
    ctx.font = `${Math.max(12, Math.floor(canvas.width * 0.08))}px sans-serif`;
    ctx.textAlign = "center";
    ctx.textBaseline = "middle";
    ctx.fillText("Loading map", canvas.width * 0.5, canvas.height * 0.5);
    return;
  }

  const bounds = minimapConfig.viewBox;
  drawPolygon(ctx, canvas, bounds, minimapConfig.outerBoundary, {
    fillStyle: "rgba(117, 22, 22, 0.18)",
    strokeStyle: "#ff4747",
    lineWidth: 2.5,
  });
  drawPolygon(ctx, canvas, bounds, minimapConfig.driveZone, {
    fillStyle: "rgba(44, 194, 82, 0.24)",
    strokeStyle: "rgba(56, 214, 95, 0.95)",
    lineWidth: 2,
  });
  drawMarkerReference(ctx, canvas, bounds, minimapConfig.markers);
  drawRcPose(ctx, canvas, bounds, latestOverlayState.rc);

  ctx.save();
  ctx.fillStyle = "#e8f3fb";
  ctx.font = `${Math.max(12, Math.floor(canvas.width * 0.07))}px sans-serif`;
  ctx.textAlign = "left";
  ctx.textBaseline = "top";
  ctx.fillText("MINIMAP", 12, 10);
  ctx.restore();
}

async function fetchOverlayState() {
  try {
    const res = await fetch("/overlay/state", { cache: "no-store" });
    if (!res.ok) {
      return;
    }
    latestOverlayState = await res.json();
    drawMinimap();
  } catch (_) {
    // Keep the last known state and try again on the next tick.
  }
}

function startOverlayPolling() {
  if (overlayStateInterval) {
    return;
  }

  overlayStateInterval = setInterval(() => {
    fetchOverlayState().catch(() => {});
  }, overlayStatePollMs);
  fetchOverlayState().catch(() => {});
}

function stopOverlayPolling() {
  if (overlayStateInterval) {
    clearInterval(overlayStateInterval);
    overlayStateInterval = null;
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
    imuEl.textContent = "IMU: waiting for phone sensor";
    if (!window.isSecureContext) {
      setStatus(waitingForImuStatus());
    }
    await ensureOrientationPermission();

    hasSensorData = false;
    orientationEventCount = 0;
    pendingInitialZero = true;
    startOrientationFeed();
    setStatus(waitingForImuStatus());

    const health = await fetch("/health");
    if (!health.ok) {
      throw new Error(`Server not ready: ${health.status}`);
    }

    connected = true;
    renderConnectButton();
    startImuAckPolling();

    sensorCheckTimer = setTimeout(() => {
      if (connected && !hasSensorData) {
        setStatus(waitingForImuStatus(true));
      }
    }, 3000);

    const token = Date.now();
    streamSource.src = `/stream.mjpg?t=${token}`;
    startRenderLoop();
    await setPtzMode("vr");
    setStatus(
      hasSensorData
        ? "VR mode active. Connecting video stream..."
        : waitingForImuStatus()
    );
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

async function initializeMinimap() {
  try {
    await loadMinimapConfig();
  } catch (_) {
    drawMinimap();
  }
  startOverlayPolling();
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
drawMinimap();
initializeMode();
initializeMinimap().catch(() => {});
window.addEventListener("resize", () => {
  if (connected) {
    resizeCanvasToDisplaySize(leftCanvas);
    resizeCanvasToDisplaySize(rightCanvas);
  }
  drawMinimap();
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
  stopOverlayPolling();
  if (connected) {
    fetch("/ptz/mode", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ mode: "manual" }),
      keepalive: true,
    }).catch(() => {});
  }
});
