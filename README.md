# Milestone 2 – Dual-Beam People Counter

## Physical Setup

Four VL53L1X (or VL53L0X) time-of-flight sensors are mounted on the door frame in a **2×2 grid**: two rows (outer and inner) with one sensor on each side of the doorway (left and right).

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

- **Left sensors** (LO, LI) are mounted on the **left side** of the door frame, pointing horizontally **across** the doorway toward the right.
- **Right sensors** (RO, RI) are mounted on the **right side**, pointing **across** toward the left.
- **Outer** sensors sit on the hallway-facing edge of the frame.
- **Inner** sensors sit on the room-facing edge of the frame.

### Mounting Dimensions

```
          Side view (looking from above)
          
    HALLWAY                          ROOM
       │                               │
       │◄──── frame depth ────►│       │
       │                       │       │
       ●  LO/RO                ●  LI/RI│
       │                       │       │
       │◄─ 100–150 mm ───────►│       │
       │         (row gap)             │
```

| Dimension | Recommended | Notes |
|-----------|-------------|-------|
| **Row gap** (outer ↔ inner) | **100 – 150 mm** | Distance between the outer and inner sensor rows along the door frame depth. This is the most critical dimension. Too close (< 60 mm) and both rows trigger simultaneously, preventing direction detection. Too far (> 200 mm) and fast walkers may clear one row before the next triggers. A standard door frame is ~100–140 mm deep, which works well — mount one sensor flush with each face. |
| **Sensor height** | **800 – 1100 mm** from floor | Roughly waist-to-chest height. Ensures detection of adults and most children. All four sensors should be at the **same height**. |
| **Doorway width** | **600 – 1200 mm** | The sensors' field of view (~25–27°) can cover a standard single door (750–900 mm). For wider openings, adjust `FAR_THRESH` upward or add more sensor pairs. |
| **Left ↔ Right** (across doorway) | Equal to **doorway width** | Left sensors mount on the left frame, right sensors on the right frame, directly facing each other across the opening. |

### Distance Zones

Each sensor measures the distance to whatever is in front of it. Three zones are defined:

| Zone | Range | Meaning |
|------|-------|---------|
| Near | 0 – 400 mm | Person is close to this sensor's side of the doorway |
| Mid | 400 – 800 mm | Person is in the center of the doorway |
| Far | 800 – 1200 mm | Person is near the opposite side of the doorway |
| None | > 1200 mm | No person detected |

These thresholds may need tuning based on your doorway width.

## How Counting Works

Direction is determined by **which row triggers first**:

| First Row Triggered | Then Second Row Triggers | Event |
|---------------------|--------------------------|-------|
| Outer (LO/RO) | Inner (LI/RI) | **Entry** (+1) — person walked in from hallway |
| Inner (LI/RI) | Outer (LO/RO) | **Exit** (−1) — person walked out to hallway |
| Outer only, then clears | Inner never triggers | No count — person turned around |
| Inner only, then clears | Outer never triggers | No count — person turned around |

### Sensor Compatibility

The firmware auto-detects the sensor model at boot:
1. Tries **VL53L1X** first
2. Falls back to **VL53L0X** if L1X init fails
3. All four sensors must be the same model

## Wiring

| Sensor | XSHUT Pin | I2C Address (assigned) |
|--------|-----------|------------------------|
| LI (index 0) | GPIO 18 | 0x2A |
| LO (index 1) | GPIO 16 | 0x2B |
| RI (index 2) | GPIO 17 | 0x2C |
| RO (index 3) | GPIO 19 | 0x2D |

All sensors share the same I2C bus (SDA/SCL). XSHUT pins are used to bring sensors out of reset one at a time during initialization so each can be assigned a unique I2C address.

## Dependencies

- [Pololu VL53L1X Arduino Library](https://github.com/pololu/vl53l1x-arduino)
- [Pololu VL53L0X Arduino Library](https://github.com/pololu/vl53l0x-arduino)
- ESP32 Arduino core (for `Preferences`, WiFi)
