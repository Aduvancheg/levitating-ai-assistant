# БЭКЛОГ ЗАДАЧ ИНТЕГРАЦИИ ИИ GEMINI PRO И USB-АУДИО v1.0
## AI Agent & Audio Integration Backlog (.docs/backlog_refactoring_ai_agent_v1.md)

### TASK-AI-1: Gemini API Async Client & Long-Term Memory (RPi 5)
- **Цель:** Внедрить асинхронный клиент Gemini Pro API (`google-generativeai` / WebSocket stream) в `main_server-v11.py`.
- **Реализация:** Добавить класс `GeminiAIManager`, SQLite базу `ai_history.db`, сохранение API-ключа и флага активации в `/etc/antigravity/ai_config.json`.
- **DoD:** Данные диалога сохраняются между перезапусками сервера.

### TASK-AI-2: Tool Calling Integration & Telemetry Context Injector
- **Цель:** Интегрировать функции управления полётом (`set_levitation_height`, `inspect_surface`, `execute_safe_shutdown`, `clear_temp_cache`).
- **Реализация:** Зарегистрировать JSON-схемы инструментов в Gemini API. Реализовать подмешивание телеметрии `SystemState` раз в 2 секунды.
- **DoD:** ИИ успешно отвечает на вопрос «Как дела?» и наклоняет сферу при вопросе «Что на столе?».

### TASK-AI-3: SPU-WM30 USB Audio Driver & VAD Loop
- **Цель:** Настроить запись и воспроизведение звука через USB-микрофон SPU-WM30.
- **Реализация:** Внедрить `AudioService` (ALSA/sounddevice + `webrtcvad`). Захват 16 кГц PCM, потоковая передача в Gemini Live API.
- **DoD:** Слышимость голоса пользователя на расстоянии до 3 метров, воспроизведение ответа через динамик SPU-WM30.

### TASK-AI-4: UI Page "AI Assistant" & 3-Step Bring-Up Panel
- **Цель:** Создать новую вкладку `AI Assistant` в `index_html_template-v11.txt`.
- **Реализация:** Добавить свитчер `Activate AI`, поле `GEMINI_API_KEY`, аудио-визуализатор волны, консоль мыслей ИИ и виджет 3-этапной проверки (`TEST-AI-01..03`).
- **DoD:** Удобное управление подключением и визуальная индикация статуса `CONNECTED & SYNCED`.
