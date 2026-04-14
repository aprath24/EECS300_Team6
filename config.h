#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

// ========== Pin Configuration ==========
#define BUTTON_PIN 0
const uint8_t xshutPins[4] = {18, 19, 16, 17};  // assign any free GPIOs

// Sensor index aliases
enum { LI = 0, LO = 1, RI = 2, RO = 3 };

// ========== Distance Thresholds (mm) ==========
// Support for VL53L1X, VL53L1CX, and VL53L0X
// Tune these for your specific doorway width
const uint16_t NEAR_THRESH   = 200;   // 0-400 mm = near zone
const uint16_t MID_THRESH    = 400;   // 400-800 mm = mid zone
const uint16_t FAR_THRESH    = 800;   // 800-1200 mm = far zone; >1200 = no detection

// Any reading below this threshold is considered a detection
const uint16_t DETECT_THRESH = FAR_THRESH;

// ========== Timing & Robustness (ms) ==========

// A row must stay active for this many ms to count as a real trigger
const unsigned long DEBOUNCE_MS = 40;

// If only one row triggers and the second never does, reset after this window
// (Handles cases where a person walks into the frame and backs out)
const unsigned long PARTIAL_TIMEOUT = 1500;

// Min time between two events in the SAME direction (prevents double counts)
const unsigned long COOLDOWN_MS = 300;

// Fully reset the state machine when nobody is detected for this long
const unsigned long INACTIVITY_TIMEOUT = 1500;

// Two beams activating within this window are treated as a simultaneous
// crossing (one entering, one exiting) → net 0, no count.
// Tune by logging raw timeDiff values for your typical door-crossing speed.
const unsigned long SIMULTANEOUS_THRESH_MS = 80;

#endif // CONFIG_H
