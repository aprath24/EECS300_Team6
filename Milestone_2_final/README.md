# Milestone 2 – Dual-Beam People Counter

## Overview

A bi-directional people counter built with four VL53L1X/VL53L0X time-of-flight sensors, an ESP32 server, and an ESP32 client with an OLED display. The system detects how many people are inside a room by tracking entries (+1) and exits (−1) through a doorway.

### Architecture

- **Server board** (`Milestone_2_final/`): Runs the ToF sensors and counting logic. Hosts a WiFi AP (`team6` / `66666666`) and serves the current count + raw sensor distances to any connected client.
- **Client board** (`Milestone_2_client/`): Connects to the server's WiFi, polls for the count, and displays it on a 128×64 OLED. A boot button sends a reset command (`#0`) to zero the count.

## Physical Setup

Four ToF sensors are mounted on the door frame in a **2×2 grid**: two rows (outer and inner) with one sensor on each side (left and right).

```
            HALLWAY (outside)
         ┌──── doorway ────┐
    LO  ●━━━━━━━━━━━━━━━━━━● RO      ← Outer row (hallway-side of frame)
         │                  │
    LI  ●━━━━━━━━━━━━━━━━━━● RI      ← Inner row (room-side of frame)
         └──────────────────┘
             ROOM (inside)
```

### Sensor Placement

| Sensor | Label | Position | Points toward |
|--------|-------|----------|---------------|
| LO | Left Outer | Left frame, hallway side | Right wall |
| RO | Right Outer | Right frame, hallway side | Left wall |
| LI | Left Inner | Left frame, room side | Right wall |
| RI | Right Inner | Right frame, room side | Left wall |

### Mounting Dimensions

| Dimension | Recommended | Notes |
|-----------|-------------|-------|
| **Row gap** (outer ↔ inner) | **100 – 150 mm** | Distance between outer and inner sensor rows along the frame depth. Too close (<60 mm) and both rows trigger together; too far (>200 mm) and fast walkers clear one before the next triggers. A standard door frame depth (~100–140 mm) works well. |
| **Sensor height** | **800 – 1100 mm** | Waist-to-chest height. Ensures detection of adults and most children. All four sensors at the **same height**. |
| **Doorway width** | **600 – 1200 mm** | Sensors' FoV (~25–27°) covers a standard single door. For wider openings increase `FAR_THRESH`. |

## How Counting Works

### Dual Per-Side State Machines

The system runs **two independent state machines** — one for the **left** sensor pair (LO + LI) and one for the **right** pair (RO + RI). Each side independently detects direction and produces a count event (+1 entry or −1 exit).

This architecture enables the system to handle scenarios that a single combined state machine cannot, such as two people passing shoulder-to-shoulder in opposite directions.

### Direction Detection

Direction is determined by **which sensor on a given side triggers first**:

| First Sensor | Then Second Sensor | Event |
|---|---|---|
| Outer (LO or RO) | Inner (LI or RI) | **Entry** (+1) — person walked in from hallway |
| Inner (LI or RI) | Outer (LO or RO) | **Exit** (−1) — person walked out to hallway |
| Only one triggers, then clears | Other never triggers | **No count** — person turned around |

### State Machine States

Each side goes through these states independently:

```
IDLE ──► OUTER_FIRST ──► BOTH_ACTIVE ──► count + WAIT_CLEAR ──► IDLE
  │                          ▲
  └──► INNER_FIRST ──────────┘
```

| State | Meaning |
|-------|---------|
| `IDLE` | No detection on this side. Waiting for a beam break. |
| `OUTER_FIRST` | Outer sensor triggered first → potential entry. Waiting for inner. |
| `INNER_FIRST` | Inner sensor triggered first → potential exit. Waiting for outer. |
| `BOTH_ACTIVE` | Both sensors active. Waiting for one to clear to confirm direction. |
| `WAIT_CLEAR` | Count was just recorded. Waiting for **all** beams to clear before accepting new triggers. Prevents tailgating false counts. |

### Merging Both Sides

After each loop cycle, the events from both sides are batched into a single net count:

```
netCount = leftDir + rightDir
```

This ensures:
- **Two people same direction**: both sides fire +1 → net +2 ✓
- **Two people opposite directions**: +1 + (−1) → net 0 ✓
- **Single person**: one side fires ±1, other side 0 → net ±1 ✓

A global cooldown (`COOLDOWN_MS = 300 ms`) suppresses rapid duplicate events in the same direction to prevent double-counting a single person.

### Robustness Features

| Feature | How it works |
|---------|-------------|
| **Debounce** (40 ms) | Each sensor must be continuously active for 40 ms before it counts as a real trigger. Filters momentary noise. |
| **Partial timeout** (1.5 s) | If only one beam triggers and the second never does, reset to IDLE after 1.5 s (person backed out). |
| **WAIT_CLEAR** | After a count event, the side waits for both beams to fully clear before re-arming. Prevents a tailgater's residual signal from being misread as a new event in the wrong direction. |
| **Inactivity timeout** (1.5 s) | If nothing is detected for 1.5 s, force IDLE on that side. |
| **Cooldown** (300 ms) | Suppresses duplicate same-direction events fired faster than 300 ms. |
| **Sensor clamping** | Readings above `DIST_MAX_CAP` (2333 mm) are treated as "no detection" to filter garbage values (e.g., 65535 from misaligned sensors). |
| **Watchdog** (5 s) | ESP32 hardware watchdog restarts the system if the main loop hangs. |

## Handled Scenarios

### Baseline (target ≥ 90% accuracy)

| Scenario | Result |
|----------|--------|
| Single person slow walk in / out, ≥3 s gap | ✅ Clean direction detection per side |
| Single person fast walk in / out, ≥3 s gap | ✅ Debounce (40 ms) still catches beam order |
| Multiple people same direction, ~1.5 m spacing | ✅ 1.5 m ≈ 1 s gap, well above cooldown (300 ms) |
| Multiple people random directions, no overlap | ✅ Each person tracked independently per side |

### Edge Cases (target ≥ 75% accuracy)

| Scenario | Result |
|----------|--------|
| Two people same direction, brushing shoulders | ✅ Both sides count: left +1, right +1 = +2 |
| Two people opposite direction, brushing shoulders | ✅ Left +1, right −1 = net 0 |
| Partial entry then reverse | ✅ Only one beam triggers → partial timeout, no count |
| Solo wheelchair | ⚠️ Both sides fire: +2 instead of +1 (over-count by 1) |
| Wheelchair pushed by another person | ✅ Wheelchair blocks both sides (+2), pusher absorbed by WAIT_CLEAR = +2 for 2 people |
| Person standing near doorway while another walks through | ✅ Standing person cycles partial timeout on one side; other side counts the walker independently |

## Configuration (`config.h`)

| Parameter | Value | Description |
|-----------|-------|-------------|
| `DETECT_THRESH` | 600 mm | Distance below which a sensor reading counts as "person detected" |
| `DIST_MAX_CAP` | 2333 mm | Readings above this are clamped to "no detection" |
| `DEBOUNCE_MS` | 40 ms | Sensor must be active this long to confirm detection |
| `PARTIAL_TIMEOUT` | 1500 ms | Reset if only one beam triggers and the second never does |
| `COOLDOWN_MS` | 300 ms | Min time between same-direction events |
| `INACTIVITY_TIMEOUT` | 1500 ms | Force IDLE when nothing detected for this long |

## Wiring

### Server Board (ToF Sensors)

| Sensor | XSHUT Pin | I2C Address (assigned) |
|--------|-----------|------------------------|
| LI (index 0) | GPIO 18 | 0x2A |
| LO (index 1) | GPIO 19 | 0x2B |
| RI (index 2) | GPIO 16 | 0x2C |
| RO (index 3) | GPIO 17 | 0x2D |

All sensors share the same I2C bus (SDA/SCL). XSHUT pins bring sensors out of reset one at a time during init so each gets a unique I2C address. Boot button (GPIO 0) resets the count to zero.

### Client Board (OLED Display)

| Component | Pin |
|-----------|-----|
| OLED SDA | GPIO 21 |
| OLED SCL | GPIO 22 |
| Boot Button | GPIO 0 (reset request) |

## WiFi Protocol

The server runs as a WiFi access point. The client connects and exchanges single-line text messages:

| Direction | Format | Meaning |
|-----------|--------|---------|
| Client → Server | `\n` (empty) | Read request — just get the current count |
| Client → Server | `#0\n` | Set count to 0 (reset) |
| Client → Server | `+\n` | Increment count by 1 |
| Client → Server | `-\n` | Decrement count by 1 |
| Server → Client | `#<count>,<LI>,<LO>,<RI>,<RO>\n` | Current count + raw sensor distances (mm) |

## Sensor Compatibility

The firmware auto-detects the sensor model at boot:
1. Tries **VL53L1X** first (using Adafruit library)
2. Falls back to **VL53L0X** if L1X init fails
3. All four sensors must be the same model
4. System continues running with however many sensors initialize successfully (minimum 1)

## Dependencies

- [Adafruit VL53L1X Arduino Library](https://github.com/adafruit/Adafruit_VL53L1X)
- [Adafruit VL53L0X Arduino Library](https://github.com/adafruit/Adafruit_VL53L0X)
- [Adafruit SSD1306](https://github.com/adafruit/Adafruit_SSD1306) (client only)
- [Adafruit GFX](https://github.com/adafruit/Adafruit-GFX-Library) (client only)
- ESP32 Arduino core (for `Preferences`, `WiFi`, `esp_task_wdt`)

## Non-Volatile Storage

The people count is saved to ESP32 flash (`Preferences` library) every 5 seconds. On reboot, the last saved count is restored automatically.
