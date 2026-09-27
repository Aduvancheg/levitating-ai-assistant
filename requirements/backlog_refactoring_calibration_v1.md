# БЭКЛОГ ЗАДАЧ: КАЛИБРОВОЧНЫЙ СТЕНД И FSM ВЗЛЕТА V1.0 (.docs/backlog_refactoring_calibration_v1.md)

## Список ключевых задач для Antigravity IDE:

* **`TASK-CAD-01` (Split Cradle Model):** Создать и скомпилировать OpenSCAD файл `calibration_stand_v1.scad` для двух половинок стенда высотой 30 мм, чашей под сферу 70 мм и штифтами стыковки.
* **`TASK-LID-V8` (Outer Locking Slots):** Обновить чертеж крышки базы до `Base_Top_Lid_V8` с 4 внешними L-пазами на радиусе `56.5 мм`.
* **`TASK-FSM-SERVER-V10` (Calibration State Machine):** Внедрить в `main_server-v10.py` конечный автомат `CalibrationFSM` со состояниями `PARKED`, `ZEROING`, `TAKEOFF_HOVER`, `WAIT_REMOVAL`, `UNLOCKED_FLIGHT` и 30-секундным таймаутом безопасности.
* **`TASK-UI-WIZARD-V10` (Interactive Modal Dialog):** Внедрить в `index_html_template-v10.txt` пошаговый мастер калибровки Calibration Wizard с визуальной подсказкой по разведению половинок подставки.
* **`TASK-TEST-FSM` (Pytest Validation):** Создать юнит-тесты в `test_calibration_fsm.py` для верификации запрета динамических полетов до вызова `confirm_stand_removed`.
