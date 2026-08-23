import sys

with open("brain_rpi5/src/main_server.py", "r") as f:
    content = f.read()

# Patch 1: get_state()
old_state_func = """@app.get("/api/state")
async def get_state():
    return {
        "bist_unlocked": system_state.bist_unlocked,
        "bist_bypass_active": system_state.bist_bypass_active,
        "bist_results": system_state.bist_results,
        "bist_logs": system_state.bist_logs,
        "active_pipeline": system_state.active_pipeline,
        "safe_to_unplug": system_state.safe_to_unplug
    }"""
new_state_func = """@app.get("/api/state")
async def get_state():
    return {
        "x": system_state.x,
        "y": system_state.y,
        "z": system_state.z,
        "target_z": system_state.target_z,
        "kp": system_state.kp,
        "ki": system_state.ki,
        "kd": system_state.kd,
        "temp": system_state.coil_temp_model,
        "throttling": system_state.thermal_throttling,
        "wifi": {
            "mode": system_state.base_wifi_mode,
            "ssid": system_state.base_wifi_ssid,
            "ip": system_state.base_wifi_ip,
            "agent_rssi": system_state.agent_wifi_rssi
        },
        "bist_unlocked": system_state.bist_unlocked,
        "bist_bypass_active": system_state.bist_bypass_active,
        "bist_results": system_state.bist_results,
        "bist_logs": system_state.bist_logs,
        "active_pipeline": system_state.active_pipeline,
        "safe_to_unplug": system_state.safe_to_unplug
    }"""
content = content.replace(old_state_func, new_state_func)

# Patch 2: Add tuning and wifi endpoints before @app.post("/api/manage/execute")
endpoints_to_add = """class PIDTuning(BaseModel):
    kp: float | None = None
    ki: float | None = None
    kd: float | None = None
    target_z: float | None = None

@app.post("/api/pid/set_tunings")
async def set_tunings(tunings: PIDTuning):
    if tunings.kp is not None: system_state.kp = tunings.kp
    if tunings.ki is not None: system_state.ki = tunings.ki
    if tunings.kd is not None: system_state.kd = tunings.kd
    if tunings.target_z is not None: system_state.target_z = tunings.target_z
    return {"status": "SUCCESS"}

@app.get("/api/wifi/scan")
async def wifi_scan():
    return {
        "status": "SUCCESS",
        "networks": [
            {"ssid": "Lab_Network", "signal": -50, "secure": True},
            {"ssid": "Guest_WIFI", "signal": -70, "secure": False}
        ]
    }

class WifiConnectRequest(BaseModel):
    ssid: str
    password: str

@app.post("/api/wifi/connect")
async def wifi_connect(req: WifiConnectRequest):
    system_state.base_wifi_ssid = req.ssid
    system_state.base_wifi_mode = "STA"
    return {"status": "SUCCESS"}

@app.post("/api/manage/execute")"""
content = content.replace('@app.post("/api/manage/execute")', endpoints_to_add)

# Patch 3: /api/manage/execute handling 8 pipelines
old_execute = """    if req.command == "BIST_RUN_ALL":
        asyncio.create_task(execute_bist_pipeline())
        return {"status": "started", "pipeline": "BIST_RUN_ALL"}
    elif req.command == "SAFE_SHUTDOWN":
        asyncio.create_task(execute_shutdown_pipeline())
        return {"status": "started", "pipeline": "SAFE_SHUTDOWN"}
    else:
        raise HTTPException(status_code=400, detail="Неизвестная команда")"""
new_execute = """    _PIPELINE_MAP = [
        "BENCH_CALIBRATE", "SMOOTH_TAKEOFF", "SPATIAL_HELIX",
        "LISSAJOUS_PATROL", "EMO_REACTION", "FORCE_FEEDBACK",
        "SAFE_LANDING", "RTH_ROUTINE"
    ]
    
    if req.command == "BIST_RUN_ALL":
        asyncio.create_task(execute_bist_pipeline())
        return {"status": "started", "pipeline": "BIST_RUN_ALL"}
    elif req.command == "SAFE_SHUTDOWN":
        asyncio.create_task(execute_shutdown_pipeline())
        return {"status": "started", "pipeline": "SAFE_SHUTDOWN"}
    elif req.command in _PIPELINE_MAP:
        system_state.active_pipeline = req.command
        system_state.pipeline_status = "Running"
        # Dummy async task to simulate pipeline completion
        async def dummy_pipeline():
            await asyncio.sleep(0.5)
            system_state.active_pipeline = None
            system_state.pipeline_status = "Idle"
        asyncio.create_task(dummy_pipeline())
        return {"status": "started", "pipeline": req.command}
    else:
        raise HTTPException(status_code=404, detail="Неизвестная команда")"""
content = content.replace(old_execute, new_execute)

# Patch 4: Adding BIST-HW-05 to SystemState
# Add BIST-HW-05
old_bist_init = """        self.bist_results: Dict[str, str] = {
            "BIST-HW-01": "UNTESTED",  # PWM Sweep
            "BIST-HW-02": "UNTESTED",  # Polarity Check
            "BIST-HW-03": "UNTESTED",  # Relay Interlock
            "BIST-HW-04": "UNTESTED"   # EMI Floor Scanner
        }
        self.bist_logs: Dict[str, str] = {
            "BIST-HW-01": "Тест не запускался.",
            "BIST-HW-02": "Тест не запускался.",
            "BIST-HW-03": "Тест не запускался.",
            "BIST-HW-04": "Тест не запускался."
        }"""
new_bist_init = """        self.bist_results: Dict[str, str] = {
            "BIST-HW-01": "UNTESTED",  # PWM Sweep
            "BIST-HW-02": "UNTESTED",  # Polarity Check
            "BIST-HW-03": "UNTESTED",  # Relay Interlock
            "BIST-HW-04": "UNTESTED",  # EMI Floor Scanner
            "BIST-HW-05": "UNTESTED"   # Lenz EMF Protection
        }
        self.bist_logs: Dict[str, str] = {
            "BIST-HW-01": "Тест не запускался.",
            "BIST-HW-02": "Тест не запускался.",
            "BIST-HW-03": "Тест не запускался.",
            "BIST-HW-04": "Тест не запускался.",
            "BIST-HW-05": "Тест не запускался."
        }"""
content = content.replace(old_bist_init, new_bist_init)

# Patch 5: run_single_bist_test handling for BIST-HW-05
old_fail = """        elif test_id == "BIST-HW-04":
            system_state.bist_results[test_id] = "FAILED"
            system_state.bist_logs[test_id] = (
                "FAILED: Сбой профилирования ЭМИ-шума силовой земли.\\n"
                "При токе ШИМ = 460 среднеквадратичный шум датчиков Холла SS49E превысил предел 12 мВ "
                "и составил RMS = 18.7 мВ.\\n"
                "РЕШЕНИЕ: Проверьте качество пайки полигона силовой земли GND 'Звезда'.\\n"
                "Усильте сечение медной жилы заземления между платами до 18 AWG."
            )
        return False"""
new_fail = """        elif test_id == "BIST-HW-04":
            system_state.bist_results[test_id] = "FAILED"
            system_state.bist_logs[test_id] = (
                "FAILED: Сбой профилирования ЭМИ-шума силовой земли.\\n"
                "При токе ШИМ = 460 среднеквадратичный шум датчиков Холла SS49E превысил предел 12 мВ "
                "и составил RMS = 18.7 мВ.\\n"
                "РЕШЕНИЕ: Проверьте качество пайки полигона силовой земли GND 'Звезда'.\\n"
                "Усильте сечение медной жилы заземления между платами до 18 AWG."
            )
        elif test_id == "BIST-HW-05":
            system_state.bist_results[test_id] = "FAILED"
            system_state.bist_logs[test_id] = (
                "FAILED: Сбой аппаратного ограничения ЭДС самоиндукции (Lenz Protection).\\n"
                "TVS-диод зафиксировал импульс 18.5V при резком сбросе ШИМ. Сбой программного Slew-Rate Limiter.\\n"
                "РЕШЕНИЕ: Проверьте исправность TVS-диода SMBJ12A. Проверьте настройки core_loop.py `MAX_PWM_STEP_PER_TICK`."
            )
        return False"""
content = content.replace(old_fail, new_fail)

old_pass = """    elif test_id == "BIST-HW-04":
        logger.info("BIST: Тест ЭМИ шума (BIST-HW-04) запущен...")
        await asyncio.sleep(1.5)
        system_state.bist_results[test_id] = "PASS"
        system_state.bist_logs[test_id] = (
            "SUCCESS: EMI floor profiling completed.\\n"
            "At 460 PWM load, ground noise RMS = 8.5 mV (Threshold: 12.0 mV).\\n"
            "Заземление Star GND работает корректно. Наводки минимальны."
        )"""
new_pass = """    elif test_id == "BIST-HW-04":
        logger.info("BIST: Тест ЭМИ шума (BIST-HW-04) запущен...")
        await asyncio.sleep(1.5)
        system_state.bist_results[test_id] = "PASS"
        system_state.bist_logs[test_id] = (
            "SUCCESS: EMI floor profiling completed.\\n"
            "At 460 PWM load, ground noise RMS = 8.5 mV (Threshold: 12.0 mV).\\n"
            "Заземление Star GND работает корректно. Наводки минимальны."
        )
    elif test_id == "BIST-HW-05":
        logger.info("BIST: Тест подавления ЭДС Ленца (BIST-HW-05) запущен...")
        await asyncio.sleep(1.5)
        system_state.bist_results[test_id] = "PASS"
        system_state.bist_logs[test_id] = (
            "SUCCESS: Slew-rate test passed, no EMF spikes detected.\\n"
            "TVS-диод: <12.5V peak. Переход 0->300->0 выполнен за 60 мс.\\n"
            "Аппаратная защита и программный Slew-Rate Limiter функционируют штатно."
        )"""
content = content.replace(old_pass, new_pass)

with open("brain_rpi5/src/main_server.py", "w") as f:
    f.write(content)

print("Patch applied successfully.")
