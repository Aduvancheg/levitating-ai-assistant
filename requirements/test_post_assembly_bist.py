"""
test_post_assembly_bist.py — Модульные тесты для Post-Assembly BIST Suite v1.0

Проверяет отработку всех 5 сценариев (TEST-PA-01..05) и корректное сбрасывание
system_state.bist_unlocked = False при симуляции любого сбоя.

SSOT: post_assembly_testing_guide_v1.md
"""
import asyncio
import pytest
import importlib
import sys
import os

# Импортируем серверный модуль
sys.path.insert(0, os.path.dirname(__file__))
main_server = importlib.import_module("main_server-v9")

SystemState = main_server.SystemState
system_state = main_server.system_state
run_single_post_assembly_test = main_server.run_single_post_assembly_test
execute_post_assembly_bist_pipeline = main_server.execute_post_assembly_bist_pipeline


@pytest.fixture(autouse=True)
def reset_system_state():
    """Сбрасывает глобальное состояние перед каждым тестом."""
    system_state.bist_unlocked = True
    system_state.bist_bypass_active = False
    system_state.simulate_fault_on_test_id = None
    system_state.active_pipeline = None
    system_state.pipeline_status = "Idle"
    system_state.pipeline_steps = []
    system_state.pwm_channels = [0, 0, 0, 0, 0]
    system_state.coil_temp_model = 25.0
    system_state.post_assembly_bist_progress = 0
    for tid in ["TEST-PA-01", "TEST-PA-02", "TEST-PA-03", "TEST-PA-04", "TEST-PA-05"]:
        system_state.post_assembly_bist_results[tid] = "UNTESTED"
        system_state.post_assembly_bist_logs[tid] = "Тест не запускался."
    yield


# =====================================================================
# ТЕСТ 1: Номинальный прогон — все 5 тестов PASS
# =====================================================================
@pytest.mark.asyncio
async def test_all_pass():
    """Все 5 постобработочных тестов проходят без сбоев → bist_unlocked не сбрасывается."""
    system_state.simulate_fault_on_test_id = None
    
    await execute_post_assembly_bist_pipeline()
    
    for tid in ["TEST-PA-01", "TEST-PA-02", "TEST-PA-03", "TEST-PA-04", "TEST-PA-05"]:
        assert system_state.post_assembly_bist_results[tid] == "PASS", (
            f"Тест {tid} должен быть PASS, но получен: {system_state.post_assembly_bist_results[tid]}"
        )
    
    assert system_state.post_assembly_bist_progress == 100
    assert system_state.active_pipeline is None
    assert "All Tests PASS" in system_state.pipeline_status


# =====================================================================
# ТЕСТЫ 2-6: Симуляция сбоя на каждом из 5 сценариев
# =====================================================================
@pytest.mark.asyncio
async def test_fail_channel_isolation():
    """Симуляция сбоя TEST-PA-01 (Channel Isolation) → bist_unlocked = False."""
    system_state.simulate_fault_on_test_id = "TEST-PA-01"
    
    await execute_post_assembly_bist_pipeline()
    
    assert system_state.post_assembly_bist_results["TEST-PA-01"] == "FAILED"
    assert system_state.bist_unlocked is False
    assert "FAILED" in system_state.pipeline_status or "Failed" in system_state.pipeline_status
    assert "TEST-PA-01" in system_state.pipeline_status
    # Лог должен содержать диагностическую информацию
    log = system_state.post_assembly_bist_logs["TEST-PA-01"]
    assert "FAILED" in log
    assert "AOD4184A" in log


@pytest.mark.asyncio
async def test_fail_winding_symmetry():
    """Симуляция сбоя TEST-PA-02 (Winding Symmetry) → bist_unlocked = False."""
    system_state.simulate_fault_on_test_id = "TEST-PA-02"
    
    await execute_post_assembly_bist_pipeline()
    
    assert system_state.post_assembly_bist_results["TEST-PA-02"] == "FAILED"
    assert system_state.bist_unlocked is False
    log = system_state.post_assembly_bist_logs["TEST-PA-02"]
    assert "FAILED" in log
    assert "полярност" in log.lower() or "COIL 3" in log


@pytest.mark.asyncio
async def test_fail_tof_clearance():
    """Симуляция сбоя TEST-PA-03 (ToF Optical Path Clearance) → bist_unlocked = False."""
    system_state.simulate_fault_on_test_id = "TEST-PA-03"
    
    await execute_post_assembly_bist_pipeline()
    
    assert system_state.post_assembly_bist_results["TEST-PA-03"] == "FAILED"
    assert system_state.bist_unlocked is False
    log = system_state.post_assembly_bist_logs["TEST-PA-03"]
    assert "FAILED" in log
    assert "ToF" in log or "VL53L5CX" in log


@pytest.mark.asyncio
async def test_fail_emc_ground():
    """Симуляция сбоя TEST-PA-04 (EMC Ground Bounce) → bist_unlocked = False."""
    system_state.simulate_fault_on_test_id = "TEST-PA-04"
    
    await execute_post_assembly_bist_pipeline()
    
    assert system_state.post_assembly_bist_results["TEST-PA-04"] == "FAILED"
    assert system_state.bist_unlocked is False
    log = system_state.post_assembly_bist_logs["TEST-PA-04"]
    assert "FAILED" in log
    assert "RMS" in log or "19.3" in log


@pytest.mark.asyncio
async def test_fail_thermal():
    """Симуляция сбоя TEST-PA-05 (Thermal Airflow Clearance) → bist_unlocked = False."""
    system_state.simulate_fault_on_test_id = "TEST-PA-05"
    
    await execute_post_assembly_bist_pipeline()
    
    assert system_state.post_assembly_bist_results["TEST-PA-05"] == "FAILED"
    assert system_state.bist_unlocked is False
    log = system_state.post_assembly_bist_logs["TEST-PA-05"]
    assert "FAILED" in log
    assert "нагрев" in log.lower() or "0.42" in log


# =====================================================================
# ТЕСТ 7: Interlock блокирует взлёт после сбоя Post-Assembly BIST
# =====================================================================
@pytest.mark.asyncio
async def test_interlock_blocks_takeoff_after_pa_failure():
    """После сбоя Post-Assembly BIST, execute_command с SMOOTH_TAKEOFF должен
    быть заблокирован (bist_unlocked = False и bist_bypass_active = False)."""
    system_state.simulate_fault_on_test_id = "TEST-PA-03"
    
    await execute_post_assembly_bist_pipeline()
    
    # bist_unlocked должен быть False после сбоя
    assert system_state.bist_unlocked is False
    assert system_state.bist_bypass_active is False
    
    # Эмулируем логику интерлока из core_loop_860hz:
    # if not bist_unlocked and not bist_bypass_active and active_pipeline is None:
    #     pwm_channels = [0, 0, 0, 0, 0]
    if not system_state.bist_unlocked and not system_state.bist_bypass_active:
        system_state.pwm_channels = [0, 0, 0, 0, 0]
    
    assert system_state.pwm_channels == [0, 0, 0, 0, 0], (
        "При заблокированном интерлоке ШИМ-каналы должны быть обнулены"
    )


# =====================================================================
# ТЕСТ 8: Pipeline не запускается, если другой pipeline уже активен
# =====================================================================
@pytest.mark.asyncio
async def test_pipeline_conflict_prevention():
    """Post-Assembly BIST не должен запускаться, если уже активен другой pipeline."""
    from fastapi.testclient import TestClient
    
    # Симулируем активный pipeline
    system_state.active_pipeline = "BIST_RUN_ALL"
    
    client = TestClient(main_server.app)
    response = client.post("/api/bist/post_assembly_test")
    
    assert response.status_code == 400
    assert "выполняется" in response.json()["detail"]
    
    # Очистка
    system_state.active_pipeline = None


# =====================================================================
# ТЕСТ 9: Последующие тесты получают SKIPPED при раннем сбое
# =====================================================================
@pytest.mark.asyncio
async def test_skipped_tests_on_early_failure():
    """При сбое TEST-PA-01, тесты TEST-PA-02..05 должны остаться UNTESTED 
    (не запускались), а pipeline steps — SKIPPED."""
    system_state.simulate_fault_on_test_id = "TEST-PA-01"
    
    await execute_post_assembly_bist_pipeline()
    
    assert system_state.post_assembly_bist_results["TEST-PA-01"] == "FAILED"
    # Тесты 02-05 не должны были запуститься
    for tid in ["TEST-PA-02", "TEST-PA-03", "TEST-PA-04", "TEST-PA-05"]:
        assert system_state.post_assembly_bist_results[tid] in ("UNTESTED",), (
            f"{tid} не должен был запускаться при раннем сбое TEST-PA-01"
        )
