# Tricorder, Bill of Materials

Everything needed to build one tricorder. Prices are what I paid at time of order and will drift, so treat them as a guide. Several items are multipacks or assortment kits where you only use a fraction, so the real per build cost is lower than the column totals suggest.

Adafruit items can be found at adafruit.com/product/[PID].

## Brain and inputs

| Item | Qty | Purpose | Source | ~Price |
|------|-----|---------|--------|--------|
| ESP32-S3 (N16R8, 16MB flash / 8MB PSRAM) | 1 | Main microcontroller | Amazon | ~$9 |
| Rotary Encoder + Extras (PID 377) | 1 | Mode / menu navigation | Adafruit | $4.50 |
| Tactile push button assortment (6x6mm / 7x7mm) | 1 kit | UI buttons (a few used) | Amazon | $8.99 |

Note: any N16R8 ESP32-S3 board works. I ended up with a couple of variants, you only need one.

## Sensors

| Item | Qty | Measures | Source | ~Price |
|------|-----|----------|--------|--------|
| BME688 Temp / Humidity / Pressure / Gas (PID 5046) | 1 | Environmental + VOCs | Adafruit | $19.95 |
| LTR390 UV Light Sensor (PID 4831) | 1 | UV | Adafruit | $4.95 |
| LIS2MDL Triple-axis Magnetometer (PID 4488) | 1 | Compass / heading | Adafruit | $7.95 |
| VL53L4CX Time of Flight Distance Sensor (PID 5425) | 1 | Distance | Adafruit | $14.95 |
| APDS9999 Proximity / Light / Color Sensor (PID 6461) | 1 | Color | Adafruit | $7.50 |
| AMG8833 IR Thermal Camera Breakout (PID 3538) | 1 | Thermal imaging | Adafruit | $44.95 |
| PDM Microphone Breakout (PID 4346) | 1 | Sound analysis / recording | Adafruit | $4.95 |
| PCA9548 8-Channel I2C Multiplexer (PID 5626) | 1 | Bus expander for all I2C sensors | Adafruit | $6.95 |

## Display and output

| Item | Qty | Purpose | Source | ~Price |
|------|-----|---------|--------|--------|
| 1.8" TFT LCD, ST7735S, 128x160, w/ SD slot | 1 | Main screen + onboard file storage | Amazon | $9.48 |
| Mini 8x8 LED Matrix w/ I2C Backpack (PID 959) | 1 | Per-mode animations | Adafruit | $11.95 |
| STEMMA Speaker, Plug and Play Amp (PID 3885) | 1 | Audio out | Adafruit | $5.95 |

## Power

| Item | Qty | Purpose | Source | ~Price |
|------|-----|---------|--------|--------|
| 3.7V 3000mAh LiPo, JST PH2.0 | 1 | Battery | Amazon | $12.49 |
| TP4056 charge / DC-DC boost board | 1 (10pk) | Charging + boost | Amazon | $11.99 |

Note: this charge/boost board is the part responsible for the occasional won't-power-on issue, since it is really designed for a power bank. Functional, but the weak point if you want to improve the build.

## Cables and connectors

| Item | Qty | Purpose | Source | ~Price |
|------|-----|---------|--------|--------|
| STEMMA QT / JST SH 4-pin cable (mix of 50mm and 200mm) | ~7 | One per I2C sensor, plus mux to ESP32 | Adafruit (PID 4399 / 4401) | ~$0.95–1.25 ea |
| STEMMA QT to premium female sockets, 150mm (PID 4397) | 1 | Wires the PDM mic to GPIO | Adafruit | $0.95 |
| STEMMA JST PH 2mm 3-pin to female socket (PID 3894) | 1 | Wires the speaker to GPIO | Adafruit | $1.25 |
| JST PH 2mm 4-pin vertical connector (10-pack) (PID 4390) | 1 pk | Custom connections (e.g. matrix) | Adafruit | $3.50 |

Note on quantities: I over-ordered cables. You need roughly 7 STEMMA QT cables total (one per I2C sensor plus one linking the multiplexer to the ESP32), use the 200mm ones for parts that sit far from the mux in the enclosure and 50mm for the close ones. You only need 1 of the PH 3-pin cables for the speaker (I bought 2). Buying one or two spares is still smart since the connectors are easy to damage.

## Enclosure and hardware

| Item | Qty | Purpose | Source | ~Price |
|------|-----|---------|--------|--------|
| PLA filament, 1.75mm (~1kg) | 1 | 3D printed enclosure | Amazon | $11.99 |
| M2 screw / nut / washer assortment | 1 kit | Assembly | Amazon | $6.99 |

## Discrete components

These are assortment kits. Only a handful of values are actually used in the build, but they are worth having on hand.

| Item | Qty | Purpose | Source | ~Price |
|------|-----|---------|--------|--------|
| Resistor assortment kit (1/4W, 1%) | 1 kit | Pull-ups / current limiting | Amazon | $4.99 |
| Capacitor assortment kit | 1 kit | Decoupling / smoothing | Amazon | $9.99 |
| 3mm LED assortment | 1 kit | Indicator LEDs | Amazon | $6.99 |

## Rough total

Buying everything fresh lands around $230 to $250, but the assortment kits and the 10-pack boost board mean a single build uses only a fraction of several line items, so your effective cost per tricorder is lower.
