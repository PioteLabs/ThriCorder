# ThriCorder Wiring Guide

All wiring for the ESP32-S3 (N16R8) build. I2C sensors all run through a PCA9548A multiplexer — see the MUX section before wiring sensors.

---

## Power

### Components
- 3.7V LiPo battery (3000mAh, JST PH 2.0)
- TP4056 + boost converter combo board (charges via USB, boosts to 5V out)
- Slide or rocker switch

### Wiring
```
LiPo (+) ──→ BMS B+
LiPo (-) ──→ BMS B-

BMS OUT+ ──→ Switch pin 1
Switch pin 2 ──→ 5V rail

BMS OUT- ──→ GND rail
```

The 5V rail powers the ESP32 (via the 5V/VIN pin), TFT, and any sensor that needs 5V.
The 3.3V rail for sensors comes from the ESP32's onboard 3.3V regulator.

> **Note:** The TP4056 boost board is the weak point of this build — it is designed for power banks and occasionally won't power on from a cold start. Press the power button twice if this happens. A dedicated LiPo charger + separate boost converter is a cleaner solution if you want to improve it.

---

## ESP32-S3 Pin Assignments

| GPIO | Connected to |
|------|-------------|
| 5    | I2C SDA (to MUX, then all sensors) |
| 6    | I2C SCL (to MUX, then all sensors) |
| 7    | TFT DC |
| 8    | TFT RST |
| 9    | TFT CS |
| 10   | SPI MOSI (TFT + SD) |
| 11   | SPI SCK (TFT + SD) |
| 12   | SPI MISO (SD) |
| 13   | PDM Mic CLK |
| 14   | PDM Mic DATA |
| 19   | SD card CS |
| 20   | Rotary encoder SW (push) |
| 21   | Button RIGHT |
| 38   | Speaker signal (PWM audio) |
| 39   | Button LEFT |
| 40   | Button BOTTOM |
| 41   | Rotary encoder CLK |
| 42   | Rotary encoder DT |
| 47   | LED group (single LED or strip, via LEDC PWM) |
| 48   | Button TOP |

All buttons and the encoder SW wire: one leg to the GPIO, other leg to GND. Internal pull-ups are enabled in firmware — no external resistors needed.

---

## I2C Multiplexer (PCA9548A)

The PCA9548A default I2C address is **0x70**. This build uses **0x71** to avoid conflicts.

### Address change — bridge A0
- Solder a wire or blob from the **A0** pad to **VCC (3.3V)**
- Leave **A1** and **A2** unconnected (pulled LOW internally)
- Result: address = 0x70 + 0b001 = **0x71**

### MUX connections
```
MUX VIN  ──→ 3.3V
MUX GND  ──→ GND
MUX SDA  ──→ GPIO 5
MUX SCL  ──→ GPIO 6
```

### Channel assignments

| MUX Channel | Sensor |
|-------------|--------|
| CH0 | 8x8 LED Matrix (HT16K33) |
| CH1 | BME688 (temp / humidity / pressure / gas) |
| CH2 | LIS2MDL (magnetometer) |
| CH3 | LTR390 (UV) |
| CH5 | APDS-9250 (color / light) |
| CH6 | AMG8833 (thermal camera) |
| CH7 | VL53L4CX (time of flight / distance) |

Each sensor connects to the matching SD/SC pair on the MUX (SC0/SD0 through SC7/SD7). Power each sensor from 3.3V and GND directly from the rail — not through the MUX.

---

## TFT Display (ST7735S, 1.8", 128×160)

| TFT Pin | Connects to |
|---------|------------|
| VCC     | 3.3V |
| GND     | GND |
| CS      | GPIO 9 |
| RESET   | GPIO 8 |
| DC/RS   | GPIO 7 |
| SDA/MOSI| GPIO 10 |
| SCK     | GPIO 11 |
| LED/BL  | 3.3V (backlight always on) |

The TFT's onboard SD slot shares the SPI bus:

| SD Pin  | Connects to |
|---------|------------|
| SD CS   | GPIO 19 |
| MOSI    | GPIO 10 (shared) |
| SCK     | GPIO 11 (shared) |
| MISO    | GPIO 12 |

---

## Speaker

Uses the Adafruit STEMMA Speaker with built-in amp (PID 3885). Wire via the JST PH 3-pin connector:

| STEMMA Speaker Pin | Connects to |
|-------------------|------------|
| Signal            | GPIO 38 |
| VIN               | 5V |
| GND               | GND |

---

## PDM Microphone (Adafruit PID 4346)

| Mic Pin | Connects to |
|---------|------------|
| 3V      | 3.3V |
| GND     | GND |
| CLK     | GPIO 13 |
| DAT     | GPIO 14 |
| SEL     | GND (selects left channel) |

---

## 8x8 LED Matrix (Adafruit mini, HT16K33, PID 959)

Connects via the MUX on **CH0**.

```
Matrix VCC ──→ 3.3V
Matrix GND ──→ GND
Matrix SDA ──→ MUX SD0
Matrix SCL ──→ MUX SC0
```

Default I2C address is **0x70**. Since the MUX isolates it on its own channel there is no address conflict — no bridging needed on the matrix itself.

---

## Rotary Encoder (Adafruit PID 377)

| Encoder Pin | Connects to |
|-------------|------------|
| CLK (A)     | GPIO 41 |
| DT (B)      | GPIO 42 |
| SW (push)   | GPIO 20 |
| VCC         | 3.3V |
| GND         | GND |

> Add 100nF ceramic capacitors from CLK→GND and DT→GND at the encoder pins to suppress ground-loop noise if you get spurious encoder ticks over USB.

---

## Buttons (4×, tactile 6mm)

| Button | GPIO | Other leg |
|--------|------|-----------|
| TOP    | 48   | GND |
| BOTTOM | 40   | GND |
| LEFT   | 39   | GND |
| RIGHT  | 21   | GND |

---

## LED Group

A group of LEDs wired in parallel on GPIO 47, PWM-controlled for brightness animations (pulse, strobe, etc.). Not individually addressable — they all act as one.

```
GPIO 47 ──→ resistor ──→ all LED anodes (tied together)
all LED cathodes (tied together) ──→ GND
```

Use a resistor value appropriate for your LED count and forward voltage. A 100Ω resistor works for 2–4 standard 3mm LEDs in parallel on 3.3V.
