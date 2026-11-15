# XIAO nRF52840 Sense / Seeed nRF52 1.1.13 — direct SAADC

## Why this version
The previous sketch compiled but failed to link `nrfx_saadc_*` functions. The core exposes header declarations, but does not link the corresponding driver implementations. This sketch uses `NRF_SAADC` hardware registers and `SAADC_IRQHandler` directly, avoiding all `nrfx_saadc`/TIMER/PPI symbols. No changes to Arduino core files or SDK config are needed.

## Hardware
- MAX4466 OUT -> A0 / D0 (P0.02, AIN0)
- MAX4466 VCC -> 3V3, GND -> GND
- 100 nF + 1 uF capacitors in parallel across microphone supply
- Onboard LSM6DS3TR-C at I2C 0x6A, nominal IMU poll 200 Hz

## Setup
1. Seeed nRF52 Boards (non-mbed), version 1.1.13. Select XIAO nRF52840 Sense.
2. Install Seeed Arduino LSM6DS3 library. Bluefruit and TinyUSB come with core.
3. Flash `.ino`; open Serial Monitor 115200 and confirm `SAADC direct-register timer ready 8000 Hz` and BLE ready.
4. Serve `index.html` on localhost (e.g. `python -m http.server 8000`), use Chrome/Edge, Pair, Start and Stop.
5. Export raw WAV, inspect for noise and compare to BLE-off baseline.

## Limitations and checks
- This firmware has not been compiled on your exact toolchain or tested on physical hardware.
- Blocks of 64 samples are rearmed in the SAADC interrupt; each block boundary may create a small timing discontinuity. The nominal WAV rate is 8 kHz, but actual effective sampling frequency must be measured against a stable test tone.
- Don't use `analogRead()` while this driver owns SAADC.
- Sequence counters detect queue overflows and missing BLE packets but cannot independently detect missed ADC conversions.
- 50 ms BLE batching is application-level scheduling, not a BLE connection interval guarantee.
- A0 must remain within 0–3.3 V.
