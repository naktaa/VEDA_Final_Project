const statusEl = document.getElementById("status");
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
let baseline = { pitch: 0, roll: 0, yaw: 0 };
let latestRaw = { pitch: 0, roll: 0, yaw: 0 };
let renderRaf = 0;
let reconnectTimer = null;
let hasSensorData = false;
let sensorCheckTimer = null;
let orientationEventCount = 0;

function setStatus(msg) {
  statusEl.textContent = msg;
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
  setStatus("0점 조정 완료");
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
      imuEl.textContent = "IMU: 센서값 없음 (브라우저 설정 확인)";
      return;
    }

    imuEl.textContent =
      `IMU pitch:${value.pitch.toFixed(1)} ` +
      `roll:${value.roll.toFixed(1)} yaw:${value.yaw.toFixed(1)}`;
  };

  window.addEventListener("deviceorientation", orientationHandler, true);

  sendInterval = setInterval(() => {
    if (!connected || !hasSensorData) {
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
      imuAckEl.textContent =
        `Server IMU pitch:${Number(body.pitch).toFixed(1)} ` +
        `roll:${Number(body.roll).toFixed(1)} ` +
        `yaw:${Number(body.yaw).toFixed(1)} ` +
        `t:${body.t} servo:${body.servo_ready ? "ready" : "off"} ` +
        `src:${body.source ?? "-"}`;
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
    return;
  }

  connectBtn.disabled = true;
  setStatus("Requesting sensor permission...");

  try {
    if (!window.isSecureContext) {
      setStatus("주의: HTTP 환경이라 자이로가 차단될 수 있음");
    }
    await ensureOrientationPermission();

    const health = await fetch("/health");
    if (!health.ok) {
      throw new Error(`Server not ready: ${health.status}`);
    }

    connected = true;
    hasSensorData = false;
    orientationEventCount = 0;
    zeroCalibrate();
    startOrientationFeed();
    startImuAckPolling();

    sensorCheckTimer = setTimeout(() => {
      if (connected && !hasSensorData) {
        if (!window.isSecureContext) {
          setStatus("자이로 차단됨: HTTPS 또는 insecure-origin 설정 필요");
        } else if (orientationEventCount === 0) {
          setStatus("자이로 이벤트 없음: 센서 권한/시스템 센서 토글 확인");
        } else {
          setStatus("자이로 값이 null: 브라우저 센서 정책으로 차단됨");
        }
      }
    }, 3000);

    const token = Date.now();
    streamSource.src = `/stream.mjpg?t=${token}`;
    startRenderLoop();
    setStatus("Connecting video stream...");
  } catch (err) {
    setStatus(`Error: ${err.message}`);
    disconnect();
  } finally {
    connectBtn.disabled = false;
  }
}

function disconnect() {
  stopOrientationFeed();
  stopRenderLoop();
  connected = false;
  streamSource.src = "";
  if (reconnectTimer) {
    clearTimeout(reconnectTimer);
    reconnectTimer = null;
  }
  imuAckEl.textContent = "Server IMU: -";
  clearCanvases();
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
window.addEventListener("resize", () => {
  if (connected) {
    resizeCanvasToDisplaySize(leftCanvas);
    resizeCanvasToDisplaySize(rightCanvas);
  }
});
connectBtn.addEventListener("click", connect);
centerBtn.addEventListener("click", zeroCalibrate);
fullscreenBtn.addEventListener("click", toggleFullscreen);
scrollBtn.addEventListener("click", scrollToVideo);
document.addEventListener("fullscreenchange", updateFullscreenButton);
window.addEventListener("beforeunload", disconnect);
