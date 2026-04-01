# QT CCTV Monitoring System Integrated Technical Documentation

## 1. Overview

This document describes the overall architecture and runtime behavior of the Qt-based control room system located in `C:\Users\1-14\real_final\main_window`, together with the Raspberry Pi side services that provide map calibration, RTSP metadata forwarding, RuView blind-zone inference, and RC tank streaming/control.

The system is a federation of loosely-coupled subsystems joined primarily by MQTT and secondarily by RTSP, RTP/UDP, and HTTP.

Main subsystems:

- Qt application: operator UI, event triage, map overlay, calibration, clip playback, RC command publishing
- CCTV source: Hanwha RTSP live stream
- RTSP metadata forwarder: extracts camera metadata and republishes normalized MQTT JSON
- Calibration / mapping pipeline: computes image-to-world homography from ArUco markers and publishes map/homography over MQTT
- RuView pipeline: fuses RF/CSI blind-zone inference and optional vision pose output, then publishes blind-zone events and pose via MQTT
- RC tank pipeline: tank control commands via MQTT, tank video via RTSP or low-latency RTP/UDP depending on URL
- Clip infrastructure: CCTV events may generate short evidence clips consumed by Qt via clip URLs

At runtime, the Qt application subscribes to `wiserisk/#` and converts heterogeneous JSON payloads into a common internal representation:

- `MqttEvent`
- `PoseFrame`
- `MapData`
- `QTransform`

These are rendered into:

- event list
- human bounding boxes
- pose skeleton overlay
- map polygon / robot / target marker overlay
- RuView zone state
- RC tank status and controls

---

## 2. Technology Stack

### 2.1 Qt side

The Windows control UI is implemented with:

- Qt Widgets
- OpenCV for homography solve support
- GStreamer for video receive/decode
- Mosquitto C client library (`libmosquitto`) for MQTT pub/sub

### 2.2 MQTT broker

The broker used by the Qt application is Mosquitto-compatible MQTT on:

- Host: `192.168.100.10`
- Port: `1883`

Qt subscriber topic filter:

```text
wiserisk/#
```

Qt publish topics include:

- `wiserisk/commands/vision_pose_bridge`
- `wiserisk/rc/control`
- `wiserisk/rc/goal`
- `wiserisk/calib/homography`
- `wiserisk/clips/forward`

The Pi-side reports indicate that some upstream publishers use a local Mosquitto broker on `127.0.0.1:1883`, then bridge or expose events to the LAN-visible broker consumed by the UI. In practice, MQTT is the canonical control and metadata backbone.

---

## 3. Main Qt Runtime Architecture

The main window owns these key runtime objects:

- `MqttSubscriber* m_mqtt`
- `MqttPublisher* m_pub`
- `GstVideoWidget` instances for CCTV / tank video
- `CctvOverlayWidget* m_overlay` for map and goal overlays
- `PoseOverlayWidget* m_poseOverlay` for human pose skeletons
- `HumanBoxOverlayWidget* m_humanBoxOverlay` for object-detection boxes
- clip windows / clip popup
- event list and filter UI

Broad initialization flow:

1. Build UI and responsive layout.
2. Create overlays over the central video host.
3. Load fallback map geometry and saved overlay calibration tweaks.
4. Create MQTT subscriber and publisher.
5. Subscribe to `wiserisk/#`.
6. Start publisher with client id `qt-main-goal-pub`.
7. Bind MQTT signals:
   - `eventReceived -> onMqttEvent`
   - `poseReceived -> PoseOverlayWidget`
   - `homographyReceived -> onHomographyReceived`
   - `mapReceived -> onMapReceived`
8. Start live video streams for central CCTV and tank view.

This yields a clean split:

- video path: GStreamer -> `QImage` -> widget paint
- metadata path: MQTT JSON -> typed structs -> overlay widgets / event widgets

---

## 4. Video Ingest Architecture

### 4.1 Main CCTV receive path

The default `GstVideoWidget` pipeline for a regular RTSP stream is:

```text
uridecodebin uri="rtsp://..." source::protocols=tcp source::latency=50
! queue leaky=downstream max-size-buffers=1 max-size-bytes=0 max-size-time=0
! videoconvert
! videocrop name=zcrop top=0 bottom=0 left=0 right=0
! videoscale
! video/x-raw,format=BGRA,width=<preferredOutputWidth>,pixel-aspect-ratio=1/1
! appsink name=vsink sync=false max-buffers=1 drop=true emit-signals=false
```

Design intent:

- `source::protocols=tcp`: stable RTSP receive for CCTV
- `queue leaky=downstream`: drop stale frames instead of accumulating latency
- `appsink`: Qt pulls frames itself to allow custom overlays
- `sync=false`, `drop=true`, `max-buffers=1`: low-latency display bias

### 4.2 RC tank receive path

The tank stream now supports two receive modes selected by URL string.

If URL begins with `rtsp://`, tank uses low-latency RTSP:

```text
uridecodebin uri="rtsp://..." source::protocols=udp source::latency=20
! queue leaky=downstream max-size-buffers=1 max-size-bytes=0 max-size-time=0
! videoconvert
! videocrop name=zcrop top=0 bottom=0 left=0 right=0
! videoscale
! video/x-raw,format=BGRA,width=<preferredOutputWidth>,pixel-aspect-ratio=1/1
! appsink name=vsink sync=false max-buffers=1 drop=true emit-signals=false
```

If URL is `udp://5000`, tank uses low-latency RTP/UDP:

```text
udpsrc port=5000 caps="application/x-rtp,media=video,encoding-name=H264,payload=96,clock-rate=90000"
! queue leaky=downstream max-size-buffers=1 max-size-bytes=0 max-size-time=0
! rtpjitterbuffer latency=10 drop-on-latency=true
! rtph264depay
! h264parse
! avdec_h264
! queue leaky=downstream max-size-buffers=1 max-size-bytes=0 max-size-time=0
! videoconvert
! videocrop name=zcrop top=0 bottom=0 left=0 right=0
! videoscale
! video/x-raw,format=BGRA,width=<preferredOutputWidth>,pixel-aspect-ratio=1/1
! appsink name=vsink sync=false max-buffers=1 drop=true emit-signals=false
```

Operator rule:

- `udp://5000` -> low-latency UDP receive
- `rtsp://...` -> RTSP receive
- other URL -> standard pipeline fallback

---

## 5. MQTT Subscriber Architecture

`MqttSubscriber` is a `QObject` moved to its own `QThread`. It creates a Mosquitto client and performs:

1. `mosquitto_new(...)`
2. `mosquitto_connect(host, port, 30)`
3. `mosquitto_subscribe(..., topic_, 0)`
4. loop via `mosquitto_loop(mosq_, 100, 1)`
5. reconnect on failure

The subscriber uses a broad wildcard:

```text
wiserisk/#
```

This is intentional. Topic-level selection is coarse, while semantic routing is handled after JSON parsing.

The system therefore favors one transport subscription and many in-process semantic filters.

---

## 6. MQTT Payload Classes Accepted by the UI

The subscriber accepts several JSON payload families.

### 6.1 Pose frames

`parsePoseFrame()` accepts multiple shapes.

Nested person form:

```json
{
  "camera_id": "cam01",
  "frame_w": 640,
  "frame_h": 480,
  "ts": 1710000000.0,
  "person": {
    "detected": true,
    "score": 0.85,
    "bbox": [x, y, w, h],
    "keypoints": {
      "nose": [x, y, vis],
      "left_shoulder": [x, y, vis]
    }
  }
}
```

`vision_pose` form:

```json
{
  "type": "vision_pose",
  "person_detected": true,
  "confidence": 0.92,
  "bbox": [x, y, w, h],
  "keypoints": [...]
}
```

`pose` form:

```json
{
  "type": "pose",
  "state": { "present": true, "confidence": 0.81 },
  "pose": {
    "bbox": [x, y, w, h],
    "keypoints": [ ... ]
  },
  "details": { "frame_w": 640, "frame_h": 480 }
}
```

These are converted to `PoseFrame` / `PosePerson` / `PoseKeypoint`.

### 6.2 Homography payload

Expected form:

```json
{
  "type": "homography",
  "key": "H_img2world",
  "rows": 3,
  "cols": 3,
  "data": [9 doubles]
}
```

The UI interprets this as image-to-world transform and inverts it into world-to-image for overlay rendering.

### 6.3 Map graph payload

Expected form:

```json
{
  "nodes": [{"id":"10","x":1.0,"y":330.0}, ...],
  "edges": [{"from":"10","to":"11"}, ...],
  "polyline": [{"x":1.0,"y":1.0}, ...]
}
```

The same map is rendered in minimap widgets and the CCTV overlay.

### 6.4 Generic event payloads

Everything else becomes `MqttEvent`, which stores:

- source fields: `src`, `cam`, `sensorId`, `zone`
- event identity: `messageType`, `topic`, `topicFull`, `rule`, `action`, `objectId`, `objectType`
- geometry: `bbox`, `frameW`, `frameH`
- state fields: `state`, `enabled`, `hasEnabled`
- fusion metadata: `confidence`, `level`, `activeNodes`, `detectedNodes`, `threshold`
- clip metadata: `clipUrl`, `clipSec`
- RC robot state fields: `rcConnected`, `rcX`, `rcY`, `rcHeading`, battery, mission, motors, etc.

---

## 7. MQTT Event Parsing Details

`handleMessage()` normalizes upstream JSON because publishers do not share one rigid schema.

### 7.1 Topic and source normalization

- `src` from `src` or `source`
- `cam` from `cam`, else `zone` in some cases
- `topic` from `topic`, else `event_type`, else `type`
- `topicFull` defaults to actual MQTT topic when absent

### 7.2 Time normalization

- `utc` from `utc`, else `ts`

### 7.3 Object type normalization

`objectType` is accepted from:

- `objectType`
- `object_type`
- `label`
- `class`

### 7.4 Bounding box normalization

The code accepts:

1. explicit fields
   - `bbox_left`, `bbox_top`, `bbox_right`, `bbox_bottom`
2. array forms
   - `[left, top, right, bottom]`
   - `[x, y, w, h]`
3. object forms
   - `{left, top, right, bottom}`
   - `{x, y, w, h}`
   - `{x, y, width, height}`

For array input, the parser tests both corner and size interpretations and chooses the one that better fits the frame.

### 7.5 State normalization

`state` may be:

- a boolean
- a number
- a string such as `true`, `1`, `yes`, `on`, `detected`
- a nested object with `present`, `text`, `confidence`

The parser normalizes this into:

- `ev.state : bool`
- `ev.rawState : QString`
- `ev.confidence : double`

### 7.6 Frame size normalization

`frameW`, `frameH` are accepted from aliases including:

- `frame_w`, `frame_h`
- `frameWidth`, `frameHeight`
- `source_w`, `source_h`
- `image_w`, `image_h`
- `video_w`, `video_h`
- nested `frame.{w,h,width,height}`

This is essential for correct mapping into the on-screen video rectangle.

---

## 8. Selective MQTT Consumption and UI Filtering

There are two different notions of “selective receive”.

### 8.1 Broker-level subscription selection

Broker-level filter is broad:

```text
wiserisk/#
```

The UI intentionally receives many message families.

### 8.2 In-process semantic selection

After receipt, the UI selectively accepts or ignores specific events.

Examples:

- `MotionAlarm` and `MotionDetection` are ignored in `onMqttEvent()`.
- `ObjectDetection/Human` is dropped if human detection is disabled.
- RuView events are dropped if RuView UI is disabled.
- only active / relevant event classes enter the event list.

### 8.3 Human detection gating

There is an explicit runtime gate:

- `MqttSubscriber::setHumanDetectionEnabled(bool)`

If a message is:

- `topic == ObjectDetection`
- `objectType == Human`
- and human detection is disabled,

then the subscriber returns early before the event reaches the main UI logic.

So the toggle is not only visual; it reduces handling overhead for those messages.

### 8.4 Event list filtering

The list supports additional client-side filtering by:

- event category
- search text
- RuView-only / detection-only views
- deduplication and pause behavior

There are therefore three stages:

1. wildcard MQTT subscription
2. semantic acceptance in code
3. operator-facing list filtering in UI

---

## 9. CCTV Metadata Flow

Based on the `rtsp_meta_forward` report, the Hanwha camera emits metadata on a separate RTSP track. A Pi-side forwarder extracts that metadata and republishes it as normalized MQTT JSON.

### 9.1 Upstream extraction

The report shows an ffmpeg path similar to:

```text
ffmpeg -rtsp_transport tcp -i <RTSP_URL> -map 0:1 -c copy -f data -
```

This selects the metadata track rather than the image track and streams XML to stdout.

### 9.2 Upstream normalization

The forwarder:

- parses ONVIF / VideoAnalytics XML
- extracts interesting event classes such as `IvaArea` and `ObjectDetection`
- attaches clip metadata if generated
- publishes flattened JSON over MQTT

### 9.3 Event shape arriving in Qt

Example object detection event:

```json
{
  "src":"cctv",
  "cam":"cam01",
  "topic":"ObjectDetection",
  "topic_full":"tns1:VideoAnalytics/ObjectDetection",
  "utc":"2026-03-20T06:02:23.373Z",
  "action":"HumanDetected",
  "objectId":"831906",
  "state":true,
  "clip_sec":5,
  "objectType":"Human",
  "bbox_left":1690.0,
  "bbox_top":469.0,
  "bbox_right":1775.0,
  "bbox_bottom":564.0,
  "center_x":1732.5,
  "center_y":516.5
}
```

Example IvaArea event may additionally carry:

- `clip_url`
- `clip_sec`
- rule/zone information

Qt treats these as `MqttEvent` and then:

- updates recent CCTV zone sightings
- optionally renders human boxes
- inserts an event list item
- optionally opens clip popup for IvaArea clips

---

## 10. Human Bounding Box Rendering Pipeline

This path is separate from pose skeleton rendering.

### 10.1 Trigger condition

A human bbox candidate satisfies:

- `src == cctv`
- `topic == ObjectDetection`
- `objectType == Human`

### 10.2 Acceptance filter

`shouldAcceptHumanBox()` rejects bad boxes using:

- rectangle validity
- positive width and height
- confidence threshold: reject if `confidence < 0.30` when confidence exists
- minimum width and height proportional to frame size
- aspect ratio bounds
- rejection of tiny square-ish boxes likely to be false positives

Let bbox width and height be:

\[
 w = x_{max} - x_{min}, \qquad h = y_{max} - y_{min}
\]

Let frame dimensions be \(W, H\). Then the filter uses approximately:

\[
 w \ge \max(36, 0.018W), \qquad h \ge \max(54, 0.035H)
\]

and aspect ratio bounds:

\[
 0.12 \le \frac{w}{h} \le 1.05
\]

### 10.3 Smoothing

Boxes are smoothed per `objectId` by exponential interpolation:

\[
B_t = B_{t-1} + \alpha (C_t - B_{t-1})
\]

with:

\[
\alpha = 0.35
\]

This reduces jitter without large lag.

### 10.4 Box persistence

Seen timestamps are stored and stale boxes are removed after:

- `kHumanBoxKeepAliveMs = 700`

### 10.5 Rendering geometry

Boxes are mapped from source frame coordinates into the displayed video rectangle by:

\[
 x' = x_{display,left} + s_x x,
 \qquad
 y' = y_{display,top} + s_y y
\]

where:

\[
 s_x = \frac{W_{display}}{W_{src}}, \qquad s_y = \frac{H_{display}}{H_{src}}
\]

An outward scaling is also applied around the source-frame center:

\[
 x_{adj} = c_x + (x - c_x) \cdot 1.03,
 \qquad
 y_{adj} = c_y + (y - c_y) \cdot 1.03
\]

with the same factor applied to width and height. This compensates for the slight “compressed toward center” visual effect seen during tuning.

### 10.6 Current tuned offsets

Current constants in `mainwindow.cpp`:

- `kHumanBoxOffsetXRatio = 0.0`
- `kHumanBoxOffsetYRatio = 0.035`
- `kHumanBoxPerspectiveOffsetXRatio = 0.0`

Meaning:

- slight downward source-space Y shift before mapping
- no global X shift
- no active right-side-only perspective X bias at present

### 10.7 Visual style

The overlay uses:

- green rectangle stroke
- badge text `person`
- top-of-box badge placement with inside fallback if necessary

---

## 11. Pose / Skeleton Rendering Pipeline

The pose pipeline is separate from CCTV object detection.

### 11.1 Data model

`PoseFrame` contains:

- `cameraId`
- `frameW`, `frameH`
- `ts`
- `PosePerson person`

`PosePerson` contains:

- `detected`
- `score`
- `bbox`
- `keypoints : QMap<QString, PoseKeypoint>`

Each keypoint contains:

- `pt = (x, y)`
- `visibility`
- `valid`

### 11.2 Supported pose edges

The skeleton graph is:

- `nose -> left_shoulder`
- `nose -> right_shoulder`
- `left_shoulder -> right_shoulder`
- `left_shoulder -> left_elbow`
- `left_elbow -> left_wrist`
- `right_shoulder -> right_elbow`
- `right_elbow -> right_wrist`
- `left_shoulder -> left_hip`
- `right_shoulder -> right_hip`
- `left_hip -> right_hip`
- `left_hip -> left_knee`
- `left_knee -> left_ankle`
- `right_hip -> right_knee`
- `right_knee -> right_ankle`

### 11.3 Visibility threshold

An edge is drawn only if both endpoint keypoints satisfy:

- `valid == true`
- `visibility >= 0.5`

Point markers are drawn only for keypoints with `visibility >= 0.5`.

### 11.4 Coordinate mapping

Source pose coordinates are mapped into the displayed video rectangle using the same affine scale used for video letterboxing:

\[
 x' = x_{display,left} + \frac{W_{display}}{W_{src}} x
\]
\[
 y' = y_{display,top} + \frac{H_{display}}{H_{src}} y
\]

This is a 2D scale+translate from source frame pixel space to the displayed video rectangle.

### 11.5 Rendered layers

Pose overlay renders:

1. red bbox
2. green skeleton edges
3. yellow keypoints
4. top-left label: `cameraId | Pose score`

### 11.6 Relation to RuView

RuView pose publications feed directly into `PoseOverlayWidget`. If RuView UI is disabled, the pose frame is cleared rather than displayed.

---

## 12. Map Overlay and Homography

The control room uses a world-plane map overlay projected into CCTV image space through a homography.

### 12.1 World plane definition

The default local fallback world points are:

- ID 10 -> `(1.0, 330.0)`
- ID 11 -> `(95.0, 330.0)`
- ID 12 -> `(1.0, 1.0)`
- ID 13 -> `(95.0, 1.0)`

These define a planar quadrilateral corresponding to the monitored floor region.

### 12.2 Homography math

Let image coordinates be \((u, v)\) and world coordinates be \((x, y)\). Using homogeneous coordinates:

\[
\begin{bmatrix}x \\ y \\ 1\end{bmatrix}
\sim
H_{img \to world}
\begin{bmatrix}u \\ v \\ 1\end{bmatrix}
\]

For rendering the map into the image, the UI uses the inverse transform:

\[
H_{world \to img} = H_{img \to world}^{-1}
\]

Expanded form:

\[
 x = \frac{h_{11}u + h_{12}v + h_{13}}{h_{31}u + h_{32}v + h_{33}},
 \qquad
 y = \frac{h_{21}u + h_{22}v + h_{23}}{h_{31}u + h_{32}v + h_{33}}
\]

### 12.3 Solve method

`HomographyCalib::computeWorldToImage()` calls OpenCV `findHomography(...)` on corresponding world and image points.

The Qt-side saved calibration path currently uses:

- world points = `m_calibWorldPts`
- image points = `m_calibImgPts`
- `useRansac = false` after the user explicitly places four points

The Pi-side workflow report indicates an upstream calibration tool may use:

- `cv::findHomography(imgPts, worldPts, cv::RANSAC, 3.0)`

before publishing `H_img2world`.

### 12.4 Matrix convention

Qt stores the transform in `QTransform`, while MQTT publishes a flattened 3x3 matrix. The code preserves matrix convention carefully when converting between OpenCV matrices and Qt transforms.

---

## 13. Calibration Workflow

There are two relevant calibration paths.

### 13.1 Upstream Pi ArUco calibration

From the supplied report, the Pi-side process is:

1. Observe four ArUco markers with IDs 10, 11, 12, 13 in the CCTV image.
2. Associate each marker center with known world coordinates.
3. Solve \(H_{img \to world}\).
4. Save YAML.
5. Publish `homography` payload over MQTT.
6. Publish map graph over MQTT.

This allows downstream consumers to share a common `frame = world` convention.

### 13.2 Qt capture calibration

The Qt UI also provides operator-driven calibration.

Step 1: capture frame from the current central video.

Step 2: choose seed points from one of:

- previously saved calibration points
- projection of world corners through existing homography
- a centered rectangular fallback

Step 3: edit four image-space correspondences interactively in `CaptureCalibOverlay`.

Step 4: solve world-to-image homography from:

- `m_calibWorldPts`
- `m_calibImgPts`

Step 5: lock and persist calibration:

- `m_homographyLocked = true`
- save to `overlay_calib.ini`
- publish inverse homography to `wiserisk/calib/homography`

Step 6: compute reprojection RMSE:

\[
\text{RMSE} = \sqrt{\frac{1}{N} \sum_{i=1}^{N} \| \hat{p}_i - p_i \|^2}
\]

where:

- \(p_i\): saved image calibration point
- \(\hat{p}_i = H_{world \to img}(w_i)\): projected world point

This yields a direct pixel-space fit quality measure.

---

## 14. Overlay Transform Refinement

In addition to the base homography, the UI supports operator tweak parameters:

- translation: `dx`, `dy`
- uniform scale: `scale`
- rotation: `rot_deg`

These are persisted in `overlay_calib.ini` and applied as a post-transform around the overlay widget center.

If the homography originated from MQTT image coordinates, the code also composes an image-to-widget transform based on the actual displayed video rectangle. This is required because:

- the source frame may not occupy the full widget
- video is letterboxed to preserve aspect ratio
- overlay coordinates must match display-space rather than raw image-space

Thus the effective overlay transform is one of:

### Case A: MQTT homography from image space

\[
H_{effective} = T_{image \to widget} \cdot H_{world \to img}
\]

### Case B: locally calibrated widget-space transform with user tweak

\[
H_{effective} = S_{resize} \cdot T_{user} \cdot H_{world \to img}
\]

This distinction explains why map overlay alignment remains stable across resize and fullscreen changes.

---

## 15. Map Rendering and Goal Projection

`CctvOverlayWidget` draws in world space and transforms through the homography.

It can render:

- polygon/polyline of the calibrated area
- node markers
- robot pose marker and yaw line
- goal marker
- ROI rectangle

### 15.1 World-to-image projection

Every world-space point is mapped by:

\[
 p_{img} = H_{world \to img} p_{world}
\]

### 15.2 RC goal click

When camera-goal mode is enabled, a click on the CCTV image is back-projected into world coordinates by:

1. map widget click to source image point if necessary
2. invert effective homography
3. compute

\[
 p_{world} = H_{img \to world} p_{img}
\]

The resulting world point is published to MQTT:

```json
{
  "x": <world_x>,
  "y": <world_y>,
  "frame": "world",
  "ts_ms": <epoch_ms>
}
```

Topic:

```text
wiserisk/rc/goal
```

This is the core of the click-to-world targeting behavior.

---

## 16. RuView Blind-Zone Integration

RuView is the blind-spot monitoring subsystem. The supplied report describes a dual algorithm architecture combining RF/CSI inference with optional vision processing.

### 16.1 Upstream architecture summary

The Pi-side startup script launches three major processes:

1. UDP tap forwarder
2. RF/zone fusion bridge
3. backbone process with coarse CSI + vision FSM + MediaPipe

Logical chain:

- ESP32-S3 / RF nodes produce CSI/RSSI packets
- UDP fan-out duplicates packets to multiple consumers
- bridge/backbone infer person presence and zone state
- MQTT publications represent node-level, fused, coarse, and vision-pose outputs

### 16.2 RF/CSI inference concept

The report describes a presence score based on RSSI baseline deviation and motion of that deviation. In simplified form:

\[
 b_t = (1-\alpha) b_{t-1} + \alpha rssi_t
\]

\[
 \Delta_t = |rssi_t - b_{t-1}|
\]

\[
 m_t = |\Delta_t - \Delta_{t-1}|
\]

A combined person score can be modeled as:

\[
 s_t = 0.7 \cdot s_{delta} + 0.3 \cdot s_{motion}
\]

with hysteresis / streak counters to prevent flicker.

### 16.3 Qt-side interpretation of RuView messages

The Qt UI interprets RuView-related events through `isRuViewEvent(ev)`. Typical recognized types include:

- `vision_pose`
- `coarse_pose`
- fused/node level messages under RuView topics

For RuView events, Qt updates:

- RuView online/offline state
- presence boolean
- confidence
- active node count
- detected node count
- minimap zone highlight
- pose overlay when pose data is present

### 16.4 RuView control publishing

Qt can enable/disable the upstream vision pose bridge by publishing:

Topic:

```text
wiserisk/commands/vision_pose_bridge
```

Payload:

```json
{
  "type": "command",
  "target": "vision_pose_bridge",
  "command": "set_enabled",
  "value": true|false,
  "source": "qt_main_window",
  "ts": "<ISO8601 UTC>"
}
```

### 16.5 RuView UI state feedback

The UI also listens for bridge status messages of type:

```text
vision_pose_bridge_status
```

with an `enabled` field and uses that to synchronize the RuView toggle state.

### 16.6 Cross-check alerting with CCTV

When RuView detects presence in a zone without recent CCTV `IvaArea` confirmation, the UI can raise a local warning popup. This expresses an important fusion principle:

- CCTV and RuView are not identical sensors
- disagreement itself can be operationally meaningful

---

## 17. RC Tank Control and Telemetry

RC tank control uses MQTT for commands and status.

### 17.1 Control publish path

Topic:

```text
wiserisk/rc/control
```

Payload shape:

```json
{
  "type": "tank_control",
  "target": "tank",
  "group": "drive|ptz|...",
  "command": "forward|left|right|...",
  "active": true|false,
  "source": "qt_main_window",
  "ts_ms": <epoch_ms>
}
```

Buttons publish `active=true` on press and `active=false` on release.

### 17.2 Status receive path

If an MQTT message is identified as:

- `messageType == rc_status`
- or `src == rc`
- or `src == rc_car`

then it is parsed as RC status, including:

- connected state
- mode
- mission
- battery
- speed
- pose `(x, y, heading, z)`
- target coordinates
- communication state
- robot state
- per-motor torque values

This updates both summary labels and the dedicated robot status window.

---

## 18. RC Tank Video Sender Side

The RC tank sender side, as patched in the external Pi-side codebase, emits low-latency UDP H.264 RTP in addition to the legacy MJPEG/HTTP path.

Current sender pipeline:

```text
appsrc is-live=true format=time do-timestamp=true block=false
! queue leaky=downstream max-size-buffers=1 max-size-bytes=0 max-size-time=0
! videoconvert
! video/x-raw,format=NV12,width=640,height=480,framerate=20/1
! videoflip method=rotate-180
! v4l2h264enc extra-controls="controls,video_bitrate=1500000,h264_i_frame_period=5,repeat_sequence_header=1"
! video/x-h264,profile=main,level=4
! h264parse config-interval=-1
! rtph264pay pt=96 mtu=1200 config-interval=1
! udpsink host=192.168.100.10 port=5000 sync=false async=false
```

So the full RC tank live path is:

```text
Pi camera -> H264 RTP/UDP sender -> Qt UDP receiver -> BGRA appsink -> Qt paint
```

This path is intentionally separate from the main CCTV RTSP path.

---

## 19. Clip Playback and Event Evidence

The RTSP metadata forwarder can generate short evidence clips for events, especially `IvaArea`.

Qt stores clip URL and clip duration from MQTT and uses them in two main ways:

- event double-click / danger-event open
- auto popup for fresh IvaArea events when enabled

The clip popup can also publish a clip forwarding request to:

```text
wiserisk/clips/forward
```

with a payload that includes the clip URL and requester identity.

This makes the UI an operator dispatch surface for evidence clips, not just a passive viewer.

---

## 20. Event List Behavior

The event list is not a raw message log. It is an operator-focused synthesized view.

Features include:

- semantic filtering
- deduplication by event-derived key
- duplicate count aggregation
- color coding by category
- pause
- clear
- sort behavior
- automatic clip popup for specific event classes

Human object-detection events are also throttled before entering the list so dense detection frames do not flood operators.

---

## 21. Data Flow Summary by Message Family

### 21.1 Homography

Producer:
- Pi calibration publisher or Qt local calibration publish

Qt action:
- parse 3x3 matrix
- invert to world-to-image
- update overlay transform

### 21.2 Map graph

Producer:
- Pi map publisher

Qt action:
- update minimap nodes/edges/polyline
- update overlay polygon
- update calibration world points if nodes 10/11/12/13 exist

### 21.3 CCTV object detection

Producer:
- RTSP metadata forwarder

Qt action:
- parse normalized event
- if Human and enabled -> human bbox pipeline
- add filtered event list entry

### 21.4 CCTV IvaArea

Producer:
- RTSP metadata forwarder

Qt action:
- mark recent CCTV zone observation
- add event list entry
- optionally autoplay clip popup

### 21.5 RuView fused/node/coarse/vision pose

Producer:
- RuView bridge/backbone

Qt action:
- update RuView state and confidence
- render minimap zone alert
- possibly render pose skeleton
- possibly raise mismatch alert

### 21.6 RC status

Producer:
- RC robot/tank subsystem

Qt action:
- update robot status card and detail window

### 21.7 Operator commands

Producer:
- Qt

Targets:
- RuView bridge enable/disable
- RC tank control
- RC world-goal click
- clip forward requests

---

## 22. Persistence and Operator State

The UI persists multiple settings locally using `QSettings`.

Important persisted items include:

- map overlay enabled state
- auto clip popup enabled state
- overlay tweak translation / scale / rotation
- saved calibration image points
- base widget size for calibration restoration

This allows calibration and UI tuning to survive restarts.

---

## 23. Design Rationale

Several architectural choices are visible in the code.

### 23.1 MQTT as semantic backbone

MQTT was chosen because the system has many loosely-coupled producers and consumers:

- camera metadata forwarder
- calibration publisher
- RuView bridge
- RC status publisher
- Qt UI publisher/subscriber

This is a natural fit for event-driven pub/sub with small JSON payloads.

### 23.2 GStreamer + appsink instead of direct native rendering

The system intentionally decodes into app-controlled frames. This allows:

- custom Qt overlays
- calibration in widget coordinates
- custom grayscale/high-contrast/motion-highlight modes
- flexible RC tank transport choices

### 23.3 Homography-centered world model

Using a shared planar world frame gives the system a common geometric reference for:

- map drawing
- RC click-to-goal
- robot pose
- blind-zone zone semantics

### 23.4 Two human representations

The system keeps both:

- bbox-based human detection
- keypoint/skeleton pose visualization

These are not redundant. Bboxes are robust for broad detection; skeletons are richer when vision pose is available.

---

## 24. Practical Operational Summary

In normal operation, the entire stack behaves as follows.

1. The Hanwha CCTV publishes live RTSP video and metadata.
2. The Qt app receives the video over RTSP and displays it.
3. A metadata forwarder converts RTSP XML metadata into MQTT JSON.
4. The Qt app subscribes to `wiserisk/#` and normalizes incoming messages.
5. Human detections become green `person` boxes on the live view.
6. RuView blind-zone inference becomes red zone visualization and optionally pose skeletons.
7. Calibration data aligns the world map onto the camera image.
8. Operators click the image to send world-coordinate RC goals.
9. The RC tank reports back robot status via MQTT.
10. The RC tank video can be viewed either through RTSP or low-latency UDP depending on its configured URL.
11. Danger events can automatically open evidence clips.

This is a multimodal monitoring system in which:

- video gives raw situational awareness
- metadata gives machine-detected events
- homography gives spatial meaning
- RuView extends observability into CCTV blind areas
- RC tank gives an actively steerable sensor/robot endpoint

---

## 25. Current Known Constraints and Observations

### 25.1 Wildcard subscription

The app subscribes broadly and filters later. This is flexible, but not minimal from a network-message perspective.

### 25.2 Human detection false positives

Many alignment issues previously suspected to be geometry errors were actually loose upstream detections. The current UI now filters and smooths them, but detector quality remains upstream-dependent.

### 25.3 Planar assumption

Homography assumes the relevant geometry lies on a plane. Objects significantly above the floor plane or outside the calibrated area can project imperfectly.

### 25.4 Tank RTSP vs UDP behavior

Tank stream mode depends entirely on URL format. Operational misconfiguration is therefore possible if the URL is wrong even though the code path exists.

### 25.5 Pose visibility threshold

The skeleton discards keypoints below visibility 0.5. This improves visual cleanliness but may hide partially observed limbs.

---

## 26. Recommended Reading Order for Developers

For future maintenance, the most important source files are:

1. `mainwindow.cpp`
2. `MqttSubscriber.cpp`
3. `GstVideoWidget.cpp`
4. `PoseOverlayWidget.cpp`
5. `HumanBoxOverlayWidget.cpp`
6. `CctvOverlayWidget.cpp`
7. `HomographyCalib.cpp`

The supplied external reports complement these files with upstream context:

- calibration + ArUco homography workflow
- tilting VR HTTP protocol
- RuView dual algorithm report
- RTSP metadata forwarder report

---

## 27. Final System Definition

The Qt control system is best understood as a world-aware event-fusion console rather than a simple CCTV viewer.

Mathematically, it revolves around a shared world plane and transform chain:

\[
\text{video pixels} \leftrightarrow \text{image plane} \leftrightarrow \text{world plane}
\]

Semantically, it revolves around MQTT normalization:

\[
\text{heterogeneous upstream JSON/XML} \to \text{typed internal event model} \to \text{operator UI}
\]

Operationally, it fuses:

- passive CCTV observation
- blind-zone RF/vision inference
- robot telemetry and control
- evidence clip handling
- calibration-driven spatial reasoning

That combination is what makes the platform a control-room system rather than just a video player.
