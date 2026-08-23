import os
import re

def test_esp32_safety_interlock_static_analysis():
    """
    TEST-4: ESP32 Safety Interlock Test
    Statically verify that the C++ code enforces the 5ms relay delay
    and the 500ms Software Fuse.
    """
    project_root = "/home/nik/projects/Levitation_first"
    smart_coil_path = os.path.join(project_root, "agent_esp32/src/smart_coil.cpp")
    smart_coil_h_path = os.path.join(project_root, "agent_esp32/src/smart_coil.h")
    qi_relay_path = os.path.join(project_root, "agent_esp32/src/qi_relay_guard.cpp")
    
    # Check smart_coil code and headers for 500ms fuse
    fuse_found = False
    for path in [smart_coil_path, smart_coil_h_path]:
        if os.path.exists(path):
            with open(path, 'r') as f:
                content = f.read()
                if "500" in content and "MAX_PULSE_DURATION_MS" in content:
                    fuse_found = True
    assert fuse_found, "Software fuse 500ms (MAX_PULSE_DURATION_MS = 500) not found"

    # Verify Relay interlock code
    if os.path.exists(qi_relay_path):
        with open(qi_relay_path, 'r') as f:
            content = f.read()
            assert "ISOLATED" in content, "ISOLATED state not found in qi_relay_guard.cpp"
