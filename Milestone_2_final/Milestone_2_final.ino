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

// Detection threshold – any reading below this means someone is in the doorway
// (each sensor's near/mid/far zones are all below this cutoff)
const uint16_t DETECT_THRESH = FAR_THRESH;  // 1200 mm

// ========== Global Variables ==========
volatile int32_t peopleCount = 0;
volatile shared_uint32 x;
Preferences nonVol;

// Dual-beam state machine for direction detection
//   Outer row = LO, RO  (hallway side of door frame)
//   Inner row = LI, RI  (room side of door frame)
//   Entry: outer triggers first, then inner confirms
//   Exit:  inner triggers first, then outer confirms
enum DoorState {
  DOOR_IDLE,          // Nobody detected
  DOOR_OUTER_FIRST,   // Outer row triggered first → potential entry
  DOOR_INNER_FIRST    // Inner row triggered first → potential exit
};
DoorState doorState = DOOR_IDLE;
bool eventFired = false;      // has an entry/exit been counted this traversal?

// For single-file detection
unsigned long lastEventTime = 0;
int lastDirection = 0;       // 1=entry, -1=exit
const unsigned long SINGLE_FILE_WINDOW = 500; // ms

// Inactivity tracking
unsigned long lastActiveTime = 0;
const unsigned long INACTIVITY_TIMEOUT = 1000; // ms

// ========== Function Prototypes ==========
void initSensors();
void readAllDistances(uint16_t dist[4]);
bool isOuterRowActive(uint16_t dist[4]);
bool isInnerRowActive(uint16_t dist[4]);
void updateCounting(bool outerActive, bool innerActive, uint16_t dist[4]);
int estimateGroupSize(uint16_t dist[4]);
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
  
  Serial.println("Dual-beam people counter ready");
  Serial.println("Layout: LO/RO (outer row) | LI/RI (inner row)");
}

// ========== Main Loop ==========
void loop() {
  uint16_t dist[4];
  readAllDistances(dist);
  
  bool outerActive = isOuterRowActive(dist);
  bool innerActive = isInnerRowActive(dist);
  
  updateCounting(outerActive, innerActive, dist);
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

// ========== Row Detection ==========
// A row is "active" when at least one sensor on that row detects a person
bool isOuterRowActive(uint16_t dist[4]) {
  return (dist[LO] < DETECT_THRESH) || (dist[RO] < DETECT_THRESH);
}

bool isInnerRowActive(uint16_t dist[4]) {
  return (dist[LI] < DETECT_THRESH) || (dist[RI] < DETECT_THRESH);
}

// ========== Counting State Machine ==========
// Direction is determined by which row (outer vs inner) triggers first.
//   Outer first → person walking IN from hallway  → entry (+1)
//   Inner first → person walking OUT from room    → exit  (-1)
// The event is counted when the SECOND row also becomes active,
// confirming the person has crossed both beams.
void updateCounting(bool outerActive, bool innerActive, uint16_t dist[4]) {
  bool anyActive = outerActive || innerActive;
  
  // ---- Inactivity timeout: reset when nobody detected for a while ----
  if (!anyActive) {
    if (millis() - lastActiveTime > INACTIVITY_TIMEOUT) {
      doorState = DOOR_IDLE;
      eventFired = false;
    }
    return;
  }
  
  lastActiveTime = millis();
  
  // ---- State machine ----
  switch (doorState) {
    case DOOR_IDLE:
      if (outerActive && !innerActive) {
        doorState = DOOR_OUTER_FIRST;
        eventFired = false;
        Serial.println("Outer row triggered first (potential entry)");
      }
      else if (innerActive && !outerActive) {
        doorState = DOOR_INNER_FIRST;
        eventFired = false;
        Serial.println("Inner row triggered first (potential exit)");
      }
      else if (outerActive && innerActive) {
        // Both rows triggered simultaneously – very fast traversal.
        // Default to entry (more common case).
        doorState = DOOR_OUTER_FIRST;
        eventFired = false;
        Serial.println("Both rows triggered simultaneously (assuming entry)");
      }
      break;
      
    case DOOR_OUTER_FIRST:
      // Waiting for inner row to confirm entry
      if (innerActive && !eventFired) {
        // Person crossed from outer to inner → ENTRY
        countEvent(1, estimateGroupSize(dist));
        eventFired = true;
      }
      break;
      
    case DOOR_INNER_FIRST:
      // Waiting for outer row to confirm exit
      if (outerActive && !eventFired) {
        // Person crossed from inner to outer → EXIT
        countEvent(-1, estimateGroupSize(dist));
        eventFired = true;
      }
      break;
  }
  
  // After event fires and both rows clear, reset to IDLE
  if (eventFired && !outerActive && !innerActive) {
    doorState = DOOR_IDLE;
    eventFired = false;
  }
}

// Estimate group size (1 or 2) by checking if both left and right
// sensors on the active row detect someone within NEAR_THRESH
int estimateGroupSize(uint16_t dist[4]) {
  // Check both sides of the outer row
  bool leftOuter = (dist[LO] < NEAR_THRESH);
  bool rightOuter = (dist[RO] < NEAR_THRESH);
  
  // Check both sides of the inner row
  bool leftInner = (dist[LI] < NEAR_THRESH);
  bool rightInner = (dist[RI] < NEAR_THRESH);
  
  // If both left and right sensors on ANY row are in near zone,
  // there are likely 2 people side by side
  if ((leftOuter && rightOuter) || (leftInner && rightInner)) return 2;
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