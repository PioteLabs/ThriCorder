# ThriCorder — Getting Started

## Get the code

Everything is free and open source at:

**https://github.com/PioteLabs/ThriCorder**

---

## What you need on your computer

1. **VS Code** — free code editor
   Download at: https://code.visualstudio.com

2. **PlatformIO** — the tool that compiles and flashes ESP32 firmware
   - Open VS Code
   - Click the **Extensions** icon on the left sidebar (looks like 4 squares)
   - Search **PlatformIO IDE**
   - Click **Install**
   - Restart VS Code when prompted

---

## How to flash the firmware

1. Go to **https://github.com/PioteLabs/ThriCorder**
2. Click the green **Code** button → **Download ZIP**
3. Unzip the folder somewhere on your computer
4. Open VS Code → **File → Open Folder** → select the unzipped folder
5. PlatformIO will automatically install all the required libraries (takes a minute)
6. Connect your ThriCorder to your computer via USB
7. Click the **→ Upload** button at the bottom of VS Code (or press the checkmark to build first)
8. Wait for it to finish — the device will reboot and start up

---

## Sound effects

The device plays WAV files from the SD card. Copy the files from the `sounds/` folder in the repo onto your SD card at this path:

```
/tricorder/sounds/
```

If the files aren't there the device still works — it falls back to beeps.

---

## Questions?

Leave a comment on the video or open an issue on GitHub.
