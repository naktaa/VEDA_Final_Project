# TwoPipeline Test Guide

This build uses a split image pipeline in `main`.

- `tracking_frame`: grayscale + CLAHE + light sharpen for LK/EIS estimation
- `display_output`: stabilized frame + display gain/gamma/optional denoise

Output mapping:

- RTSP `/raw`: original raw frame
- RTSP `/cam`: `display_output`
- tiltVR MJPEG: `display_output`

Recommended checks:

1. Confirm `[pipeline]` values load from `config_local.ini`.
2. Compare `/raw` and `/cam` to verify only `/cam` gets display processing.
3. Open tiltVR and confirm it matches RTSP `/cam`, including the existing RB swap.
4. In low-contrast scenes, verify tracking stability does not regress.
5. If the output looks too bright, lower `display_gain` or `display_gamma`.
