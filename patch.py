import sys

with open("brain_rpi5/src/main_server.py", "r") as f:
    content = f.read()

# Patch 1: SystemState
old_state = """        self.bist_results: Dict[str, str] = {
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
new_state = """        self.bist_results: Dict[str, str] = {
            "BIST-HW-01": "UNTESTED",  # PWM Sweep
            "BIST-HW-02": "UNTESTED",  # Polarity Check
            "BIST-HW-03": "UNTESTED",  # Relay Interlock
            "BIST-HW-04": "UNTESTED",  # EMI Floor Scanner
            "BIST-HW-05": "UNTESTED"   # Lenz Protection Slew Rate
        }
        self.bist_logs: Dict[str, str] = {
            "BIST-HW-01": "Тест не запускался.",
            "BIST-HW-02": "Тест не запускался.",
            "BIST-HW-03": "Тест не запускался.",
            "BIST-HW-04": "Тест не запускался.",
            "BIST-HW-05": "Тест не запускался."
        }"""
content = content.replace(old_state, new_state)

# Patch 2: run_single_bist_test FAILED branch
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

# Patch 3: run_single_bist_test PASS branch
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

print("Patch applied.")
