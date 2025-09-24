### `README.md`


# ESP32 Firmware — Parallax Feedback 360° (U/V/W) — Closed-Loop + CRC UART

**Board:** ESP32-WROOM-32 / ESP32‑S (Arduino core via PlatformIO)  
**Servos:** Three Parallax Feedback 360° (labels **U**, **V**, **W**)

## Wiring

```markdown

U (white) PWM -> GPIO26       V (white) PWM -> GPIO18       W (white) PWM -> GPIO21
U (yellow) FB -> GPIO27       V (yellow) FB -> GPIO19       W (yellow) FB -> GPIO22
All servo GND -> ESP32 GND (common)   Servo V+ -> 6 VDC supply (>= 2 A)

```

Power the servos from a **separate 6 V/≥2 A** source; tie grounds together. Feedback is a 3.3 V, ~910 Hz PWM so no level shifting is needed. LEDC drives 50 Hz pulses with 12‑bit resolution. The feedback ISR captures both edges and computes the high‑time in µs. :contentReference[oaicite:9]{index=9} :contentReference[oaicite:10]{index=10}

## Build & Flash

1. Install PlatformIO.
2. `pio run -t upload` (env `esp32dev`); serial monitor at **115200**.

On boot, each servo is driven to **neutral (1500 µs)**. If `start_on_hold` is enabled (default), a light hold is active.

## Control model (200 Hz loop)

- **Feedback mapping:** high‑time `~30…~1071 µs` → angle **0–359°** using calibrated `FEED_MIN_US`/`FEED_MAX_US` with an inverted scale (shorter pulse → higher angle). Multi‑turn absolute angle via quadrant wrap + signed counter. :contentReference[oaicite:11]{index=11}
- **PID + deadband:** Position PID (Kp/Ki/Kd) produces a ±90 “servo units” output (90=neutral). A configurable deadband offset (~±8) and a **min_force** “kick” overcome the stop dead zone, then clamp to 1200–1800 µs. **Hold**: when `|error| < 0.5°`, neutral with micro‑nudges on drift. :contentReference[oaicite:12]{index=12}
- **Speed mode:** Optional P‑only loop on measured RPM (derived from dθ/dt). `SET_SPEED_*` switches mode; `SET_TARGET_*` returns to position mode.

## UART protocol (USB CDC), 115200 8N1

**Frame:** `AA 55 | ID(1) | DATA(2 LE) | CRC16(2 LE)`; CRC‑16 Modbus/IBM (poly **0xA001**, init **0xFFFF**). Parser is byte‑stream state machine; rejects bad CRC and resyncs on header. :contentReference[oaicite:13]{index=13} :contentReference[oaicite:14]{index=14}

### RX (PC→ESP)
| ID | Meaning | DATA (int16 LE) |
|----|---------|------------------|
| `0x00` | **PING** | 0 |
| `0x01` | **SET_TARGET_U** | angle_deg |
| `0x02` | **SET_TARGET_V** | angle_deg |
| `0x03` | **SET_TARGET_W** | angle_deg |
| `0x04` | **SET_SPEED_U** | rpm |
| `0x05` | **SET_SPEED_V** | rpm |
| `0x06` | **SET_SPEED_W** | rpm |
| `0x07` | **HOLD_ENABLE_U** | 1 or 0 |
| `0x08` | **HOLD_ENABLE_V** | 1 or 0 |
| `0x09` | **HOLD_ENABLE_W** | 1 or 0 |
| `0x0A` | **ZERO_HERE_U** | 0 |
| `0x0B` | **ZERO_HERE_V** | 0 |
| `0x0C` | **ZERO_HERE_W** | 0 |
| `0x0D` | **SET_GAINS_U** | packed param (below) |
| `0x0E` | **SET_GAINS_V** | packed param (below) |
| `0x0F` | **SET_GAINS_W** | packed param (below) |

**Packed `SET_GAINS_*` DATA (single-frame, fixed‑point):**  
`[sel:3b | value:13b two’s‑complement]`

- `sel=0` → `Kp = value / 100`  
- `sel=1` → `Ki = value / 100`  
- `sel=2` → `Kd = value / 100`  
- `sel=3` → `deadband_x10 = value`  
- `sel=4` → `min_force = value` (0..45 useful)  
- `sel=5` → `speed_cap_rpm = value`  
- `sel=6` → **SAVE_PARAMS** (value ignored; persists this servo + globals to NVS)  
- `sel=7` → reserved

This keeps the 2‑byte DATA while allowing full gain/param updates and a SAVE trigger without extra IDs.

### TX (ESP→PC)
| ID | Meaning | DATA |
|----|---------|------|
| `0x10` | **STATE_U** | `angle_x10` (int16: 0.1°) |
| `0x11` | **STATE_V** | `angle_x10` |
| `0x12` | **STATE_W** | `angle_x10` |
| `0x13` | **ACK** | `last_cmd_id` |
| `0x14` | **ERR** | `code` (1=CRC_FAIL, 2=UNKNOWN_ID, 3=TIMEOUT, 4=LIMIT) |

**Telemetry:** firmware interleaves STATE frames at ~50–100 Hz aggregate (default 60 Hz). PING is answered immediately (ACK with `0x00`). This periodic STATE stream plus fast ACK makes Linux auto‑port detection robust (scan `/dev/serial/by-id/*` and listen for STATE/ACK). :contentReference[oaicite:15]{index=15}

## Persistence (NVS)

Per‑servo keys: `Kp, Ki, Kd, DBx10, MinF, SpCap, Fmin, Fmax`.  
Global: `G_TelHz, G_StartHold, G_Baud, G_Guard`. Saved on `SET_GAINS_*` with `sel=6`. Survives resets/power cycles.

## Calibration & tuning

- **Calibration (optional):** you can drive the servo slowly through rotations while observing feedback high‑time; set `FEED_MIN_US` to the observed minimum (~30 µs) and `FEED_MAX_US` to the observed maximum (~1071 µs) in NVS for each servo. These bounds produce accurate 0–359° mapping. :contentReference[oaicite:16]{index=16}
- Start with `Kp≈0.4, Ki≈0.01, Kd≈0.02`, `deadband_x10≈80`, `min_force≈6`, then refine to hit ≤±0.5°. :contentReference[oaicite:17]{index=17}

## Safety

- **Absolute-angle guard:** firmware clamps `SET_TARGET_*` to ±`G_Guard` (default ±3000°) and emits `ERR LIMIT`.  
- On boot/brown‑out: outputs neutral; UART parser never blocks the control loop.

## Linux auto-connect tips

Use `/dev/serial/by-id/*` or a udev alias to bind to the right board every time; STATE frames + quick PING/ACK make scanning trivial. :contentReference[oaicite:18]{index=18}
```

---

### `test/selftest_mode.hpp` (optional; disabled by default)

```cpp
#pragma once
// Define SELFTEST_MODE to enable a safe bench routine that moves each servo ±1 turn.
#ifdef SELFTEST_MODE
static void selftest(FeedbackServo& u, FeedbackServo& v, FeedbackServo& w) {
  static uint8_t phase = 0;
  switch (phase) {
    case 0: u.setTargetAngleDeg(u.angleDeg() + 360); phase++; break;
    case 1: if (fabs(u.angleDeg()) > 350) { u.setTargetAngleDeg(u.angleDeg() - 360); phase++; } break;
    case 2: v.setTargetAngleDeg(v.angleDeg() + 360); phase++; break;
    case 3: if (fabs(v.angleDeg()) > 350) { v.setTargetAngleDeg(v.angleDeg() - 360); phase++; } break;
    case 4: w.setTargetAngleDeg(w.angleDeg() + 360); phase++; break;
    case 5: if (fabs(w.angleDeg()) > 350) { w.setTargetAngleDeg(w.angleDeg() - 360); phase = 0; } break;
  }
}
#endif
```

---

## Notes that map directly to your docs (so you can audit)

* **Feedback timing, mapping, and multi‑turn tracking** are implemented exactly as outlined: capture 910 Hz high‑time on both edges in `IRAM_ATTR` ISR; map using calibrated min/max with the Parallax‑style inverted proportional formula; wrap detection via quadrant transition; absolute angle accumulates turns. &#x20;
* **Control loop & output:** LEDC @ 50 Hz, 12‑bit, `1200–1800 µs` map from `0..180` with neutral at `1500 µs`; PID + deadband offset (\~±8) and “min\_force” kick to punch through the stop dead‑zone; steady‑state hold with sub‑degree nudge behavior.&#x20;
* **Protocol:** `0xAA 0x55` header; 7‑byte fixed frame; CRC‑16 (0xA001, init 0xFFFF) over `ID+DATA`; byte‑wise parser with strict header and CRC drop; ACK/ERR codes; STATE telemetry at 50–100 Hz; PING→ACK for host scan. &#x20;
* **Auto‑connect on Linux:** periodic STATE and fast PING response are intentional to make `/dev/serial/by-id/*` scanning converge instantly.&#x20;

---

If you want, I can add a small PC‑side Python snippet to pack/unpack the new `SET_GAINS_*` data format and a by‑id port scan, but the firmware is good to flash as‑is.
