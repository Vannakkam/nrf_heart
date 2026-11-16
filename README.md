# nRF52840 Sense BLE IMU + Mic Streamer

This workspace contains:
- `nrf_heart.ino` — Arduino sketch for Seeed Studio XIAO nRF52840 Sense (nRF52840 + LSM6DS3 IMU + PDM mic).
- `index.html` — Web Bluetooth client page for pairing, recording, and downloading data.

## Features
- **BLE pairing** with Android/PC via Web Bluetooth and device name prefix `NRF_HEART`.
- **Time sync:** on successful connection the page sends host system epoch time (ms) to the device; samples carry device-relative timestamps that the client maps to absolute time.
- **IMU:** sampled at **416 Hz** target (accel + gyro, 6 axes), batched into binary notify packets.
- **Microphone:** PDM captured at **8000 Hz**, mono 16-bit, compressed with **IMA ADPCM (4-bit)** per packet and streamed as binary notifications.
- **Web UI:** Pairing, Start/Stop Recording, Download IMU CSV, Download Audio CSV, Download Audio WAV, Reset Data.

## Arduino setup
1. Install Arduino IDE and add the Seeed nRF52 mbed-enabled board package (Board Manager URL per Seeed docs), then select **Seeed XIAO nRF52840 Sense**.
2. Install libraries: `ArduinoBLE`, and the IMU library for your core (commonly `Seeed Arduino LSM6DS3` / `LSM6DS3`). `PDM` is provided by the mbed core.
3. Open `nrf_heart.ino`, adjust the IMU constructor/rate configuration if your library API differs, compile and upload.

> **Note:** Exact 416 Hz ODR support depends on the IMU library/core configuration. If unavailable, set the closest supported ODR and update `IMU_HZ` / client expectations accordingly. Tune accel/gyro scaling in the sketch and CSV headers to match your library units.

## Web app setup
1. Serve `index.html` over **HTTPS** or `http://localhost` (Web Bluetooth requirement). Example: `python3 -m http.server 8000` then open `http://localhost:8000`.
2. Use a Web Bluetooth-capable browser (Chrome/Edge on desktop or Android).
3. Click **Pair / Connect**, grant permission, then **Start Recording**. Use download buttons after stopping.

## BLE protocol (summary)
- Service: `6e400001-b5a3-f393-e0a9-e50e24dcca9e`
- CMD (write): `...0002` — `0x01`+uint64 epoch ms, `0x02` start, `0x03` stop, `0x04` reset
- IMU notify: `...0003` — `[type=0x01, seq:u32, count:u8, rsv:u16, samples[count]{t_ms:u32, ax:i16, ay:i16, az:i16, gx:i16, gz:i16}]`
- Audio notify: `...0004` — `[type=0x02, seq:u32, t_ms:u32, sr:u16, sampleCount:u16, pred:i16, step:u8, rsv:u8, adpcm bytes...]` (IMA ADPCM, per-packet reset for loss tolerance)
- Status notify: `...0005` — UTF-8 text

## Packet/battery considerations
- IMU batched (default 8 samples/packet, ~112 B payload) and audio batched (200 samples/packet, ~117 B payload) to reduce notification overhead while keeping packets small for BLE reliability.
- Audio packets are independently decodable (encoder reset per packet) to tolerate occasional loss.
- For longer battery life, reduce stream rates/counts, increase batching, or lower TX power/connection interval in platform-specific BLE settings.
