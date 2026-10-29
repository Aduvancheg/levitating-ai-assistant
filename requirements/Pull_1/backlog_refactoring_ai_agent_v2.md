# Backlog & Technical Specification: AI Agent V12 Polish (Touches 1, 3, 4)
## Project: Antigravity (Phase 4 // Integrated Harmonic Levitation Complex)

---

### TASK-AI-5: Graceful Emergency Safe Landing (LiPo < 10% & Wi-Fi Heartbeat Loss)
- **Node:** Raspberry Pi 5 (`main_server-v12.py`) & ESP32-CAM Agent
- **Objective:** Programmatically prevent physical crashes and thermal runaway if sphere battery drops below 10.0% or if UDP/Wi-Fi telemetry packets are missed for > 500 ms.
- **Implementation Details:**
  1. `SystemState` tracks `agent_wifi_last_seen` (timestamp) and `agent_battery` (volts/%).
  2. In `core_loop_860hz()`: If `target_z > 0` and (`perf_counter() - agent_wifi_last_seen > 0.500` or `agent_battery < 10.0`):
     - Trigger `asyncio.create_task(execute_emergency_landing_pipeline(reason))`.
  3. `execute_emergency_landing_pipeline`:
     - Decrement `target_z` at 5.0 mm/s to 0.0 mm.
     - Smoothly reduce coil PWM channels at Slew-Rate limit (<= 10 units/tick) over 2.0s.
     - Broadcast audio alert via `AudioService` over SPU-WM30: *"Внимание! Зафиксирован критический разряд аккумулятора сферы (или потеря Wi-Fi связи). Выполняется аварийная посадка."*
     - Set `safe_to_unplug = True`.
- **Definition of Done (DoD):**
  - Zero hard physical impacts on base surface.
  - Verification that max PWM delta per 1.16ms tick is <= 10.
  - Audio alert played on SPU-WM30 speaker.
  - Telemetry broadcast `payload.shutdown.emergency_reason` emitted over `/ws/telemetry`.

---

### TASK-UI-5: Embedded Live MJPEG/RTSP Video Stream Player (OV2640 HUD Integration)
- **Node:** FastAPI Server (`main_server-v12.py`) & Web UI (`index_html_template-v12.txt`)
- **Objective:** Provide a real-time live video stream from ESP32-CAM camera on the AI Assistant UI tab with a telemetric HUD overlay (pitch, roll, yaw, height Z).
- **Implementation Details:**
  1. In `main_server-v12.py`: Add endpoint `@app.get("/api/agent/camera_stream")` returning `StreamingResponse(mjpeg_generator(), media_type="multipart/x-mixed-replace; boundary=frame")`.
  2. In `index_html_template-v12.txt`: Add `<div class="camera-stream-card">` inside Tab `AI Assistant`.
  3. Include HUD overlay showing Pitch, Roll, Yaw, Target Z, and AI Inspection Bounding Boxes.
  4. Buttons: `[TOGGLE STREAM]`, `[SNAPSHOT & INSPECT]`, `[RESOLUTION: 640x480 / 1600x1200]`.
- **Definition of Done (DoD):**
  - Stream latency < 200 ms.
  - Frame rate >= 15 FPS at VGA resolution.
  - Non-blocking execution: PID loop @ 860 Hz maintains microsecond accuracy while video is streaming.

---

### TASK-AI-6: Automated SQLite Database Compression & Zip Archiving
- **Node:** Raspberry Pi 5 (`main_server-v12.py`)
- **Objective:** Maintain RPi 5 SSD/SD storage health by automatically compressing `ai_history.db` into a timestamped ZIP archive when file size exceeds 50 MB, while keeping active dialogue thread intact.
- **Implementation Details:**
  1. In `clear_temp_cache()` method:
     - Check `os.path.getsize("/var/log/antigravity/ai_history.db")`.
     - If size > 50 MB:
       - Execute SQLite `PRAGMA wal_checkpoint(FULL)` and `VACUUM`.
       - Create compressed ZIP file `/var/log/antigravity/history_archive_{YYYY_MM_DD_HHMM}.zip`.
       - Prune records older than 60 days, retaining the most recent 1,000 dialogue turns.
       - Log result in `system_state.maintenance_logs`.
- **Definition of Done (DoD):**
  - `ai_history.db` file size reduced to < 10 MB post-archiving.
  - Active dialogue history remains fully accessible without session interruption.
  - Zip archive passes `zipfile.is_zipfile()` integrity check.
