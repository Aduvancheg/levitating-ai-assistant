# СТАНДАРТЫ КАЧЕСТВА И КОД-РЕВЬЮ V11.0 (AI & AUDIO EXTENSION)
## Quality Assurance & Code Review Guidelines (.docs/quality-v11.md)

### `RPI-CL-12` (Gemini API & Tool Calling Safety):
- [ ] Ключ `GEMINI_API_KEY` не логируется в открытом виде в консоли RPi 5.
- [ ] Все вызовы Tool Calling (`set_levitation_height`, `execute_safe_shutdown`) проверяют лимиты безопасности перед выполнением.
- [ ] При потере интернет-связи бэкенд выставляет статус `AI_DISCONNECTED` без падения ПИД-контура левитации (860 Гц).

### `RPI-CL-13` (Audio Stream & SPU-WM30 ALSA Check):
- [ ] Поток аудио-записи выполняется в отдельном асинхронном потоке (`asyncio.to_thread` / `executor`), не блокируя главный сервер.
- [ ] Уровень VAD порога срабатывания сбалансирован под акустический шум кулера 3010.

### `WEB-CL-10` (AI Assistant Page & Bring-Up Widget):
- [ ] Переключатель `Activate AI` визуально меняет цвет со серого на неоново-зеленый.
- [ ] Вкладка `AI Assistant` отображает задержку Ping и лог активных мыслей ИИ.
