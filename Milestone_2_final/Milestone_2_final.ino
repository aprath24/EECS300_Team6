#include <Wire.h>
#include <VL53L1X.h>
#include <VL53L0X.h>
#include "WirelessCommunication.h"
#include "sharedVariable.h"
#include "Preferences.h"

#define BUTTON_PIN 0

// ========== Sensor Configuration ==========
// Four sensors: Left Inner (LI), Left Outer (LO), Right Inner (RI), Right Outer (RO)
const uint8_t xshutPins[4] = {18, 16, 17, 19};  // assign any free GPIOs
VL53L1X sensorsL1[4];   // VL53L1X sensor objects
VL53L0X sensorsL0[4];   // VL53L0X sensor objects (fallback)
bool useL1X = true;      // true = using L1X, false = using L0X (set during init)
enum { LI = 0, LO = 1, RI = 2, RO = 3 };

// Distance thresholds (mm) – tune for your doorway width
const uint16_t NEAR_THRESH = 400;   // 0-400 mm = near zone
const uint16_t MID_THRESH  = 800;   // 400-800 mm = mid zone
const uint16_t FAR_THRESH  = 1200;  // 800-1200 mm = far zone; >1200 = no detection

// Logical zones
enum Zone { ZONE_NONE, LEFT_NEAR, LEFT_MID, LEFT_FAR, RIGHT_NEAR, RIGHT_MID, RIGHT_FAR };

// ========== Global Variables ==========
volatile int32_t peopleCount = 0;
volatile shared_uint32 x;
Preferences nonVol;

// Unified state machine for person tracking
enum SideState { STATE_OUTSIDE, STATE_FAR, STATE_MID, STATE_NEAR };
SideState trackState = STATE_OUTSIDE;
int moveDirection = 0;        // 1=entering (from far), -1=exiting (from near), 0=unknown
bool eventFired = false;      // has an entry/exit been counted this traversal?

// For single-file detection
unsigned long lastEventTime = 0;
int lastDirection = 0;       // 1=entry, -1=exit
const unsigned long SINGLE_FILE_WINDOW = 500; // ms

// Inactivity tracking
unsigned long lastActiveTime = 0;
const unsigned long INACTIVITY_TIMEOUT = 1000; // ms

// For debouncing zone transitions
unsigned long lastZoneChange = 0;
const unsigned long DEBOUNCE_MS = 100;

// ========== Function Prototypes ==========
void initSensors();
void readAllDistances(uint16_t dist[4]);
float getAverageLeft(uint16_t dist[4]);
float getAverageRight(uint16_t dist[4]);
Zone getZone(float avgLeft, float avgRight);
void updateCounting(Zone zone);
int estimateGroupSize();
void countEvent(int direction, int group);
void updateNonVolatile();
void updateSharedVar();

// ========== Setup ==========
void setup() {
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(400000);            // 400 kHz I2C
  initSensors();

  nonVol.begin("nonVolData", false);
  peopleCount = nonVol.getUInt("count", 0);
  
  init_wifi_task();
  INIT_SHARED_VARIABLE(x, peopleCount);
  
  Serial.println("6-zone people counter ready");
  Serial.println("Zones: Left-Near/Mid/Far, Right-Near/Mid/Far");
}

// ========== Main Loop ==========
void loop() {
  uint16_t dist[4];
  readAllDistances(dist);
  
  float avgLeft = getAverageLeft(dist);
  float avgRight = getAverageRight(dist);
  Zone zone = getZone(avgLeft, avgRight);
  
  updateCounting(zone);
  updateSharedVar();
  updateNonVolatile();
  
  delay(20);   // small delay to prevent I2C flooding
}

// ========== Sensor Initialization ==========
void initSensors() {
  // Reset all sensors via XSHUT
  for (int i = 0; i < 4; i++) {
    pinMode(xshutPins[i], OUTPUT);
    digitalWrite(xshutPins[i], LOW);
  }
  delay(10);
  
  // --- Auto-detect sensor type on the first sensor ---
  pinMode(xshutPins[0], INPUT);   // bring sensor 0 out of reset
  delay(50);
  
  // Try VL53L1X first
  sensorsL1[0].setTimeout(500);
  if (sensorsL1[0].init()) {
    useL1X = true;
    Serial.println("Detected VL53L1X sensors");
    sensorsL1[0].setAddress(0x2A);
    sensorsL1[0].startContinuous(100);
    Serial.println("Sensor 0 at address 0x2A");
  } else {
    // VL53L1X failed, try VL53L0X
    sensorsL0[0].setTimeout(500);
    if (sensorsL0[0].init()) {
      useL1X = false;
      Serial.println("Detected VL53L0X sensors");
      sensorsL0[0].setAddress(0x2A);
      sensorsL0[0].startContinuous(100);
      Serial.println("Sensor 0 at address 0x2A");
    } else {
      Serial.println("Sensor 0 init failed (neither L1X nor L0X)");
      while (1);
    }
  }
  
  // --- Init remaining sensors with the detected type ---
  for (int i = 1; i < 4; i++) {
    pinMode(xshutPins[i], INPUT);
    delay(50);
    
    bool ok = false;
    if (useL1X) {
      sensorsL1[i].setTimeout(500);
      ok = sensorsL1[i].init();
      if (ok) {
        sensorsL1[i].setAddress(0x2A + i);
        sensorsL1[i].startContinuous(100);
      }
    } else {
      sensorsL0[i].setTimeout(500);
      ok = sensorsL0[i].init();
      if (ok) {
        sensorsL0[i].setAddress(0x2A + i);
        sensorsL0[i].startContinuous(100);
      }
    }
    
    if (!ok) {
      Serial.print("Sensor ");
      Serial.print(i);
      Serial.println(" init failed");
      while (1);
    }
    Serial.print("Sensor ");
    Serial.print(i);
    Serial.print(" at address 0x");
    Serial.println(0x2A + i, HEX);
  }
}

void readAllDistances(uint16_t dist[4]) {
  for (int i = 0; i < 4; i++) {
    if (useL1X) {
      dist[i] = sensorsL1[i].read();
    } else {
      dist[i] = sensorsL0[i].readRangeContinuousMillimeters();
    }
    delay(1);   // let I2C bus settle
  }
}

float getAverageLeft(uint16_t dist[4]) {
  return (dist[LI] + dist[LO]) / 2.0f;
}

float getAverageRight(uint16_t dist[4]) {
  return (dist[RI] + dist[RO]) / 2.0f;
}

// Map average distances to logical zones
Zone getZone(float avgLeft, float avgRight) {
  // Determine left zone
  Zone leftZone = ZONE_NONE;
  if (avgLeft < NEAR_THRESH) leftZone = LEFT_NEAR;
  else if (avgLeft < MID_THRESH) leftZone = LEFT_MID;
  else if (avgLeft < FAR_THRESH) leftZone = LEFT_FAR;
  
  // Determine right zone
  Zone rightZone = ZONE_NONE;
  if (avgRight < NEAR_THRESH) rightZone = RIGHT_NEAR;
  else if (avgRight < MID_THRESH) rightZone = RIGHT_MID;
  else if (avgRight < FAR_THRESH) rightZone = RIGHT_FAR;
  
  // If both sides are active, return the one with smaller distance (closer person)
  // This avoids double counting the same person when they are in the center.
  if (leftZone != ZONE_NONE && rightZone != ZONE_NONE) {
    return (avgLeft < avgRight) ? leftZone : rightZone;
  }
  if (leftZone != ZONE_NONE) return leftZone;
  if (rightZone != ZONE_NONE) return rightZone;
  return ZONE_NONE;
}

// ========== Counting State Machine ==========
// Uses direction tracking: first appearance at FAR/MID = entering,
// first appearance at NEAR = exiting. Events fire only when
// the person completes the traversal in their initial direction.
void updateCounting(Zone zone) {
  // Debounce: ignore rapid zone changes within DEBOUNCE_MS
  static Zone lastZone = ZONE_NONE;
  if (zone != lastZone) {
    if (millis() - lastZoneChange < DEBOUNCE_MS) return;
    lastZoneChange = millis();
    lastZone = zone;
  }
  
  // Extract level from zone: 0=none, 1=far, 2=mid, 3=near
  int level = 0;
  switch (zone) {
    case LEFT_FAR:  case RIGHT_FAR:  level = 1; break;
    case LEFT_MID:  case RIGHT_MID:  level = 2; break;
    case LEFT_NEAR: case RIGHT_NEAR: level = 3; break;
    default: level = 0;
  }
  
  // ---- Handle no detection (ZONE_NONE) ----
  if (level == 0) {
    if (millis() - lastActiveTime > INACTIVITY_TIMEOUT) {
      // Finalize pending fast exit: person was exiting and reached
      // at least MID but we missed FAR (walked out very quickly)
      if (moveDirection == -1 && !eventFired &&
          (trackState == STATE_MID || trackState == STATE_FAR)) {
        countEvent(-1, 1);
      }
      // Reset state for next person
      trackState = STATE_OUTSIDE;
      moveDirection = 0;
      eventFired = false;
    }
    return;
  }
  
  lastActiveTime = millis();
  
  // ---- State machine transitions ----
  if (trackState == STATE_OUTSIDE) {
    // First detection: determine direction from which zone appeared
    eventFired = false;
    if (level == 3) {
      trackState = STATE_NEAR;
      moveDirection = -1;  // appeared at NEAR → exiting (came from inside)
    } else if (level == 2) {
      trackState = STATE_MID;
      moveDirection = 1;   // appeared at MID → entering
    } else {
      trackState = STATE_FAR;
      moveDirection = 1;   // appeared at FAR → entering
    }
  }
  else if (trackState == STATE_FAR) {
    if (level == 2) {
      trackState = STATE_MID;
    }
    else if (level == 3) {
      trackState = STATE_NEAR;
      // Entering direction reached NEAR → count entry
      if (moveDirection == 1 && !eventFired) {
        countEvent(1, estimateGroupSize());
        eventFired = true;
      }
    }
    // level==1: still at FAR, no change needed
  }
  else if (trackState == STATE_MID) {
    if (level == 3) {
      trackState = STATE_NEAR;
      // Entering direction reached NEAR → count entry
      if (moveDirection == 1 && !eventFired) {
        countEvent(1, estimateGroupSize());
        eventFired = true;
      }
    }
    else if (level == 1) {
      trackState = STATE_FAR;
      // Exiting direction reached FAR → count exit
      if (moveDirection == -1 && !eventFired) {
        countEvent(-1, estimateGroupSize());
        eventFired = true;
      }
    }
  }
  else if (trackState == STATE_NEAR) {
    if (level == 2) {
      trackState = STATE_MID;
    }
    else if (level == 1) {
      trackState = STATE_FAR;
      // Exiting direction reached FAR → count exit
      if (moveDirection == -1 && !eventFired) {
        countEvent(-1, estimateGroupSize());
        eventFired = true;
      }
    }
  }
}

// Estimate group size (1 or 2) by checking if both left and right near zones are active
int estimateGroupSize() {
  uint16_t dist[4];
  readAllDistances(dist);
  float avgLeft = getAverageLeft(dist);
  float avgRight = getAverageRight(dist);
  
  bool leftNear = (avgLeft < NEAR_THRESH);
  bool rightNear = (avgRight < NEAR_THRESH);
  
  if (leftNear && rightNear) return 2;
  return 1;
}

// Record an event (direction: +1 entry, -1 exit) with given group size
void countEvent(int direction, int group) {
  unsigned long now = millis();
  
  // Single‑file detection: if same direction within window, add extra
  if (lastDirection == direction && (now - lastEventTime) < SINGLE_FILE_WINDOW) {
    peopleCount += direction * group;
    Serial.printf("Single‑file %s: %+d (extra)\n", (direction==1?"entry":"exit"), direction*group);
  } else {
    peopleCount += direction * group;
    Serial.printf("%s: %+d\n", (direction==1?"Entry":"Exit"), direction*group);
  }
  
  if (peopleCount < 0) peopleCount = 0;
  Serial.printf("Total people inside: %d\n", peopleCount);
  
  lastDirection = direction;
  lastEventTime = now;
}

// ========== Non‑Volatile Storage ==========
void updateNonVolatile() {
  static unsigned long lastSave = 0;
  if (millis() - lastSave > 5000) {  // save every 5 seconds
    nonVol.putUInt("count", peopleCount);
    lastSave = millis();
  }
}

void updateSharedVar() {
  LOCK_SHARED_VARIABLE(x);
  x.value = peopleCount;
  UNLOCK_SHARED_VARIABLE(x);
}

// Button press (optional, kept for compatibility)
uint32_t is_pressed() {
  if (!digitalRead(BUTTON_PIN)) {
    delay(10);
    if (!digitalRead(BUTTON_PIN)) {
      while (!digitalRead(BUTTON_PIN));
      return 1;
    }
  }
  return 0;
}