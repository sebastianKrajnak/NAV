# NAV - ESP 32 Heart rate measurement

## Assignment
The main goal of the project was to create a heart measurement device using the provided kit, which featured: 
- Wemos D1 R32 ESP32 board,
- SSD1306 OLED display,
- MAX30102 Oximeter,
- KX-040 (KY-040) Rotary encoder.

The realization was done in C with the ESP-IDF framework, in PlatformIO and using these open sourced libraries:
- SSD1306 OLED Display: https://github.com/nopnop2002/esp-idf-ssd1306 
- MAX30102 Oximeter: https://github.com/Gustbel/max30102_esp-idf 
- KX-040 (KY-040) Rotary Encoder: https://github.com/nopnop2002/esp-idf-RotaryEncoder

## Start-up
The pinout is written in the included documentation and the used sdkconfig file should make sure that there is no need to change any settings,
however if the pins used in the realization are different from the ones specified in the doc, then you need to configure the new pins:
- in the menuconfig configuration menu for the OLED display
- in platformio.ini file for the oximeter.

## Points
NaN / 17
