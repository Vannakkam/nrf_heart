# XIAO nRF52840 Sense 8 kHz / 50 ms BLE recorder — Seeeduino nRF52 1.1.13

## Hardware
MAX4466 OUT -> XIAO A0; MAX4466 VCC -> 3V3; GND -> GND. Onboard LSM6DS3TR-C IMU, no external MPU6050. Put 100 nF and 1 uF in parallel at microphone VCC/GND.

## Firmware changes
- Uses **nrfx SAADC API v2** as present in Seeeduino nRF52 non-mbed core **1.1.13**.
- Removes TIMER2 and PPI dependencies, using the SAADC hardware internal timer instead (16 MHz / 2000 = 8 kHz).
- Uses `nrfx_saadc_init(6)`, `nrfx_saadc_channels_config`, `nrfx_saadc_advanced_mode_set`, `nrfx_saadc_buffer_set`, `nrfx_saadc_mode_trigger`.
- Requeues buffers when `NRFX_SAADC_EVT_BUF_REQ` occurs.
- MAX4466 A0 sample stream is retained in BLE binary notifications; 50 ms burst schedule and 200 Hz nominal IMU polling unchanged.

## Installation
Arduino Boards Manager: Seeed nRF52 Boards (non-mbed) 1.1.13; select XIAO nRF52840 Sense. Install Seeed Arduino LSM6DS3 library. Use the provided `index.html` served through http://localhost:8000. Open Serial Monitor at 115200 baud.

## Cautions and verification
This sketch targets the exact header API confirmed from Seeed 1.1.13 source, but it has not been fully compiler-tested or measured on hardware. The nrfx SAADC driver may interact with other ADC users; do not call analogRead() while recording. Some samples can be omitted at start/stop buffer boundaries; timestamps are nominal and not hardware-captured for each sample. Bluetooth throughput and speech audio quality need hardware testing. A0 must stay in the safe 0-3.3 V range.

If the compiler still reports a different error, include the first error lines and selected board definition.
