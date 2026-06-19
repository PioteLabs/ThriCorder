# ThriCorder v3.8

A handheld multi-sensor device built on the ESP32-S3, inspired by Star Trek tricorders. Measures environment, distance, color, thermal, sound, and wireless signals — displayed on a 1.8" TFT with an 8x8 LED matrix and PWM audio SFX.

> Shared as-is, not actively maintained. Build it, hack it, have fun.

## Video

🎬 *[link coming soon — check the channel]*

## What you need

See [`bom/`](bom/) for the full shopping list with links and prices.

Key parts:
- ESP32-S3 dev board with PSRAM
- 1.8" ST7735 TFT display
- Adafruit 8x8 LED matrix (HT16K33, I2C)
- BME688 (temp/humidity/pressure/gas)
- LTR390 (UV)
- LIS2MDL (magnetometer/compass)
- VL53L4CX (time-of-flight distance)
- AMG8833 (8x8 thermal camera)
- APDS-9250 (color/light, native I2C driver — not the APDS-9960)
- INMP441 or similar PDM microphone
- TCA9548A I2C multiplexer
- MicroSD card module
- Rotary encoder + 4 tactile buttons
- Small speaker (bare, driven via PWM on GPIO38)

## How to flash

1. Install [PlatformIO](https://platformio.org/)
2. Clone this repo
3. Open the folder in VS Code with the PlatformIO extension
4. Connect your ESP32-S3 via USB
5. Click **Upload**

## Sound effects

The device plays WAV files from the SD card (`/tricorder/sounds/`). Drop the files from [`sounds/`](sounds/) onto your SD card. If a file is missing it falls back to beeps automatically.

## Enclosure

Print the STL files in [`stl/`](stl/). Designed for FDM, no supports needed on most parts.

## WiFi / file browser

In **FILES** mode, hold the TOP button for 1 second to open the WiFi submenu. Host an access point and browse/download files at `http://192.168.4.1`. WiFi credentials are stored on the SD card at `/tricorder/config/wifi.json` — never in the firmware.

## License

MIT — see [LICENSE](LICENSE).
