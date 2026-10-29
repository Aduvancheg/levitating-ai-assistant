# Quality Checklists & Code Standards V12 (Polish Edition)

## RPI-CL-14 (Emergency Landing & Heartbeat Check)
- [ ] Emergency landing pipeline correctly triggers when Wi-Fi elapsed time > 500ms or battery < 10.0%.
- [ ] Descent speed is strictly 5.0 mm/s to prevent hard surface impacts.
- [ ] Slew-rate limiter limits PWM ramp down rate to <= 10 units per 1.16ms tick.
- [ ] Voice warning is dispatched over SPU-WM30 audio interface.

## WEB-CL-11 (Embedded MJPEG Live Stream)
- [ ] MJPEG video stream available at `/api/agent/camera_stream`.
- [ ] UI renders video with pitch/roll/yaw/Z HUD overlay.
- [ ] Latency remains under 200 ms without blocking the 860 Hz PID loop.

## RPI-CL-15 (SQLite Database Archiving)
- [ ] `ai_history.db` size monitored during `clean_cache` executions.
- [ ] Database automatically checkpointed, vacuumed, and archived into timestamped ZIP file if > 50 MB.
- [ ] No active dialogue loss during archiving operations.
