# MIC_IMU — ESP32-C3 audio + motion recorder

Streams a MAX4466 microphone and an MPU6050 over BLE to a Web Bluetooth page,
with host-synchronised epoch timestamps on every sample block.

- `MIC_IMU.ino` — ESP32-C3 firmware (Arduino core 3.x / NimBLE)
- `index.html` — Chrome/Edge Web Bluetooth client and CSV/WAV exporter

## Hardware

| Signal | ESP32-C3 SuperMini | Notes |
| --- | --- | --- |
| MPU6050 SDA | GPIO8 | 400 kHz I2C, address `0x68` |
| MPU6050 SCL | GPIO9 | |
| MPU6050 VCC | 3V3 | do not use 5 V logic |
| MPU6050 GND | GND | |
| MAX4466 OUT | GPIO0 | ADC1 channel 0 |
| MAX4466 VCC | 3V3 | keep the mic supply clean |
| MAX4466 GND | GND | |

The MAX4466 output sits at about VCC/2, so the firmware subtracts 2048 from the
12-bit ADC value before ADPCM encoding. Adjust the mic gain trimpot so loud
sounds do not clip: the status page shows raw ADC values if you export the
audio CSV.

## Build and flash

Tested against **esp32 core 3.3.11**:

```powershell
arduino-cli compile --fqbn esp32:esp32:esp32c3 .
arduino-cli upload  --fqbn esp32:esp32:esp32c3 --port COMx .
```

In the Arduino IDE select **ESP32C3 Dev Module**. Keep the default partition
scheme; the sketch uses about 49% of flash and 13% of RAM. USB CDC on boot can
stay at the board default (the sketch only uses `Serial` for diagnostics).

## Using the web client

Web Bluetooth only works in a secure context. Serve the page rather than
opening it with `file://`:

```powershell
python -m http.server 8000
```

Then open <http://localhost:8000/index.html> in Chrome or Edge.

1. **Pair / Connect** — pick the device named `MicIMU-C3`.
   On connect the page writes the host UTC clock to the device.
2. **Start recording** — the page sends a random 16-bit session id.
   The device streams 1-second blocks until stopped.
3. **Stop recording** — the device flushes the current block with the
   *final* flag; the page then leaves the recording state.
4. **Export**:
   - **IMU CSV** — raw axes plus g / °/s, one row per 200 Hz sample.
   - **Audio CSV** — decoded 12-bit ADC value and millivolts at 8 kHz.
   - **WAV** — 16-bit mono 8 kHz, DC-blocked and peak-normalised.
5. **Reset data** — stops the device and clears all data held in the page.

If a recording is stopped and the final block does not arrive within four
seconds, the page ends the session locally so the UI never stays stuck.

## BLE interface

Device name: `MicIMU-C3`

| Purpose | UUID |
| --- | --- |
| Service | `4fafc201-1fb5-459e-8fcc-c5c9c331914b` |
| Data (notify) | `beb5483e-36e1-4688-b7f5-ea07361b26a8` |
| Control (write) | `cba1d466-344c-4be3-ab3f-189f80dd7518` |
| Status (read/notify) | `1c95d5e3-d8f7-413a-bf3d-24e8d3c8d3e9` |

Control commands (little-endian):

| Command | Payload | Meaning |
| --- | --- | --- |
| `0x01` | `u64` unix milliseconds | set device clock |
| `0x02` | optional `u16` session id | start recording |
| `0x03` | — | stop and flush final block |
| `0x04` | — | stop and reset device counters |
| `0x05` | — | request status |

Status is a JSON object: `fw`, `rec`, `time`, `imu`, `adc`, `conn`, `mtu`,
`sub`, `sess`, `blk`, `drop`, `idrop`, `txb`, `txp`, `nerr`, `a`, `i`, `err`,
`heap`.

### Wire format

A 1-second block is 8000 audio samples + 200 IMU samples. It is split into
notifications whose size follows the negotiated ATT MTU.

Fragment header (10 bytes):

```
u8  magic = 0x4D
u8  version = 1
u16 fragmentIndex
u16 fragmentCount
u16 blockId
u16 payloadLength
```

Block body (40-byte header + payloads):

```
0   u32 magic = 0x3142494D
4   u8  version = 1
5   u8  codec = 1 (IMA ADPCM)
6   u8  flags (bit0 = final block)
7   u8  reserved
8   u16 blockId
10  u16 audioSamples
12  u16 imuSamples
14  u16 audioBytes
16  u16 imuBytes
18  u16 audioSampleRate = 8000
20  u16 imuRate = 200
22  i16 ADPCM initial predictor
24  u8  ADPCM initial step index
25  u8  reserved
26  u16 sessionId
28  u64 block start epoch microseconds
36  u32 CRC32 (header bytes 0..35 followed by the payloads)
40  audio: IMA ADPCM, low nibble first
... IMU: first sample = u16 tick + 6 × i16 raw axes;
         later samples = u16 tick delta + 6 × zig-zag LEB128 axis deltas
```

The tick unit is 125 µs (8000 ticks per second) from the session start.
Audio sample `i` in a block is timestamped `startUs + i × 125 µs`.

## Design notes

- **Hardware-timed audio.** The ADC runs in ESP-IDF continuous DMA mode at
  exactly 8 kHz instead of `analogRead()` in a loop.
- **Compression.** IMA ADPCM turns 16 kB/s of 16-bit audio into 4 kB/s. IMU
  deltas are varint-coded and usually cost well under 1 kB/s.
- **Radio noise.** Notifications are paced at one fragment every 12 ms and the
  connection interval is requested at 30–50 ms. This keeps the radio from
  saturating the audio front end. If you still hear buzz, move the antenna away
  from the mic, shorten the mic wires, and power the board from a clean supply.
- **No samples without a subscriber.** The sender only runs while the data
  characteristic is subscribed, so the BLE stream does not buffer stale data.
- **Timestamp anchor.** The host writes its clock right before each start
  command. The device records the epoch at the first audio sample and derives
  every later timestamp from the sample/tick counters.

## Troubleshooting

| Symptom | Check |
| --- | --- |
| `Pair / Connect` does nothing | Use Chrome/Edge over `http://localhost` or HTTPS; `file://` has no Web Bluetooth |
| Device not listed | Board is advertising as `MicIMU-C3`; press reset; check serial log at 115200 baud |
| Status shows `adc: 0` | GPIO0 wiring / mic power; continuous ADC failed to start |
| Status shows `imu: 0` | I2C wiring, `WHO_AM_I` should be `0x68`; try 100 kHz if wires are long |
| `err: 1` on start | Clock sync failed; reconnect so the page can write host time |
| Blocks dropped / CRC errors | Keep the phone/PC close; other 2.4 GHz traffic and a weak link cause retries |
| Audio is quiet or clipped | Adjust the MAX4466 gain trimpot; watch the audio CSV millivolt values |
| WAV has a constant DC offset | The export applies a DC-blocking filter; a large offset means the mic bias is far from VCC/2 |

## Verification performed

- Firmware compiles cleanly for `esp32:esp32:esp32c3` with core 3.3.11.
- `index.html` loads without console errors and exposes `window.__micimu`.
- A synthetic block generated to the wire format parses in the page with exact
  IMU axis/tick round-trip, valid CRC, and corruption detected after a bit flip.
- IMU CSV, audio CSV, and WAV exports were exercised end-to-end with a synthetic
  block; the WAV header reports 8 kHz, mono, 16-bit PCM and the expected data
  length.

Not verified: over-the-air behaviour on real hardware (no board attached during
development).
