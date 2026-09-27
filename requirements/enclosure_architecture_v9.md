# АРХИТЕКТУРА И ГЕОМЕТРИЯ КОРПУСА БАЗЫ V9.0 (WITH USB PASS-THROUGH)
## Enclosure Architecture & OpenSCAD Specification (.docs/enclosure_architecture_v9.md)

В версии V9.0 добавлена аппаратная поддержка прокладки USB-кабеля для конференц-микрофона **Spacetronik SPU-WM30**:

1. **Вырез `USB_AUX_PASS_THROUGH_SLOT`:**
   - Расположен на задней стенке нижнего подиума `Lower_Podium_V9` под углом **180° (South)**.
   - Размеры: Ширина **`14.0 мм`**, Высота **`8.0 мм`**, радиус фаски **`2.0 мм`**.
   - Позволяет выводить кабель от SPU-WM30 наружу к лежащему рядом микрофону без демонтажа подиума.

2. **Сохранение внешних замковых пазов V8:**
   - Сохранены 4 внешних L-паза на радиусе **`56.5 мм`** для надежной фиксации разъёмного калибровочного стенда `calibration_stand_v1.scad`.

### OpenSCAD Модуль `Lower_Podium_V9`:
```openscad
module Lower_Podium_V9() {
    difference() {
        cylinder(r=60.0, h=25.0, $fn=120);
        cylinder(r=57.0, h=23.0, $fn=120); // Внутренняя полость
        
        // Задний вырез прокладки USB-кабеля SPU-WM30
        translate([0, -58.0, 10.0]) {
            cube([14.0, 10.0, 8.0], center=true);
        }
    }
}
```
