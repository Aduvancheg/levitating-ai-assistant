# РЕГЛАМЕНТ КАЛИБРОВКИ И ПАРКОВКИ АГЕНТА V1.0 (.docs/agent_calibration_and_parking_workflow_v1.md)
## 5-Step Split Cradle Calibration & Takeoff FSM Specification

Данный документ описывает асинхронный конечный автомат (Finite State Machine / FSM) и протоколы взаимодействия между бэкендом Raspberry Pi 5 (`main_server-v10.py`), веб-интерфейсом (`index_html_template-v10.txt`) и разъемным калибровочным стендом (`calibration_stand_v1.scad`).

---

## 1. Пошаговая логика конечного автомата (Calibration FSM States)

```
 [ STATE 1: PARKED ] ──> (User presses "BENCH CALIBRATE" in UI)
          │
          ▼
 [ STATE 2: ZEROING ] ──> Zero Hall & ToF offsets @ Z = 30.0mm (RMS < 2.0mV Check)
          │
          ▼
 [ STATE 3: TAKEOFF_HOVER ] ──> PID ramps up, lifts sphere by +3.0mm to Z = 33.0mm
          │
          ▼
 [ STATE 4: WAIT_REMOVAL ] ──> UI prompts "Stand Removed?". Starts 30s Safety Timeout Task
          │                        │
          │ (User Confirms)        │ (30s Timeout Exceeded)
          ▼                        ▼
 [ STATE 5: UNLOCKED_FLIGHT ]   [ AUTO-LAND SAFE DESCENT ]
   Target Z -> 25.0mm             Target Z -> 30.0mm (Parked)
   Joystick/Helix Enabled          PID Disarmed
```

---

## 2. Спецификация REST API и WebSockets

### 1. Запуск процесса калибровки
`POST /api/calibration/start`
- **Request Body:** `{}`
- **FSM State:** `PARKED` $\to$ `ZEROING` $\to$ `TAKEOFF_HOVER` $\to$ `WAIT_REMOVAL`

### 2. Подтверждение снятия подставки
`POST /api/calibration/confirm_stand_removed`
- **Request Body:** `{}`
- **FSM State:** `WAIT_REMOVAL` $\to$ `UNLOCKED_FLIGHT`
- **Action:** ПИД-уставка плавно переводится с `Z = 33.0 мм` на рабочую высоту `Z = 25.0 мм`. Разблокируются динамические полетные функции.

### 3. Отмена / Аварийный сброс
`POST /api/calibration/cancel`
- **Request Body:** `{}`
- **Action:** Плавный опуск сферы на высоту `Z = 30.0 мм`, отключение ШИМ за 2.0 секунды.

---

## 3. Таймаут безопасности (Safety Timeout Task)

Если сфера подпрыгнула на `+3.0 мм` (Z = 33.0 мм), но пользователь в течение **30.0 секунд** не подтвердил удаление половинок подставки, бэкенд вызовет отмену калибровки:
- Автоматический плавный спуск со скоростью 5 мм/с на высоту `Z = 30.0 мм`.
- Отключение ШИМ с Slew-Rate Limiter (Ramp-down 2.0 секунды).
- Логирование в UI: `[WARNING] Calibration Timeout Exceeded (30s). Sphere returned to cradle safely.`
