import sys

with open("web_gui/index.html", "r") as f:
    content = f.read()

old_block_1 = """                            <div id="bist-log-BIST-HW-04" class="bg-slate-950 p-4 rounded-xl border border-slate-900 text-[11px] font-mono whitespace-pre-line text-slate-400 hidden">
                                Логи пусты.
                            </div>
                        </div>
                    </div>
                </div>"""

new_block_1 = """                            <div id="bist-log-BIST-HW-04" class="bg-slate-950 p-4 rounded-xl border border-slate-900 text-[11px] font-mono whitespace-pre-line text-slate-400 hidden">
                                Логи пусты.
                            </div>
                        </div>

                        <!-- BIST-HW-05: Lenz Protection -->
                        <div class="space-y-1">
                            <div onclick="toggleLogDrawer('BIST-HW-05')" class="bg-slate-950/80 p-4 rounded-xl border border-slate-900 flex justify-between items-center cursor-pointer hover:border-slate-700 transition">
                                <div class="space-y-1">
                                    <span class="text-[9px] text-slate-500 uppercase block">Test BIST-HW-05</span>
                                    <h4 class="text-xs font-bold">Lenz EMF Protection Slew-Rate</h4>
                                    <p class="text-[10px] text-slate-400">Проверка программно-аппаратного ограничения скачков напряжения при резком сбросе ШИМ.</p>
                                </div>
                                <div class="flex items-center space-x-3">
                                    <span id="bist-res-BIST-HW-05" class="px-2.5 py-1 rounded text-[10px] font-bold bg-slate-900 text-slate-500 border border-slate-800">UNTESTED</span>
                                    <button onclick="event.stopPropagation(); retrySingleTest('BIST-HW-05')" class="px-2 py-1 bg-blue-900/40 hover:bg-blue-600 text-[10px] font-bold rounded border border-blue-800 hidden" id="bist-retry-BIST-HW-05">Retry</button>
                                    <i data-lucide="chevron-down" class="w-4 h-4 text-slate-500 transition-transform duration-200" id="bist-arrow-BIST-HW-05"></i>
                                </div>
                            </div>
                            <div id="bist-log-BIST-HW-05" class="bg-slate-950 p-4 rounded-xl border border-slate-900 text-[11px] font-mono whitespace-pre-line text-slate-400 hidden">
                                Логи пусты.
                            </div>
                        </div>
                    </div>
                </div>"""

old_block_2 = """                            <option value="BIST-HW-03">BIST-HW-03: NC-Relay Interlock Fault</option>
                            <option value="BIST-HW-04">BIST-HW-04: Ground EMI Noise Fault</option>
                        </select>"""

new_block_2 = """                            <option value="BIST-HW-03">BIST-HW-03: NC-Relay Interlock Fault</option>
                            <option value="BIST-HW-04">BIST-HW-04: Ground EMI Noise Fault</option>
                            <option value="BIST-HW-05">BIST-HW-05: Lenz EMF Protection Fault</option>
                        </select>"""

content = content.replace(old_block_1, new_block_1)
content = content.replace(old_block_2, new_block_2)

# One more place!
# const ALL_BIST_TESTS = ["BIST-HW-01", "BIST-HW-02", "BIST-HW-03", "BIST-HW-04"];
old_js = 'const ALL_BIST_TESTS = ["BIST-HW-01", "BIST-HW-02", "BIST-HW-03", "BIST-HW-04"];'
new_js = 'const ALL_BIST_TESTS = ["BIST-HW-01", "BIST-HW-02", "BIST-HW-03", "BIST-HW-04", "BIST-HW-05"];'
content = content.replace(old_js, new_js)

with open("web_gui/index.html", "w") as f:
    f.write(content)
