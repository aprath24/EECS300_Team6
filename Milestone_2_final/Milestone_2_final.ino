#include <Wire.h>
#include <VL53L1X.h>
#include "WirelessCommunication.h"
#include "sharedVariable.h"
#include "Preferences.h"

#define BUTTON_PIN 0

// ========== Sensor Configuration ==========
// Four sensors: Left Inner (LI), Left Outer (LO), Right Inner (RI), Right Outer (RO)
const uint8_t xshutPins[4] = {18, 16, 17, 19};  // assign any free GPIOs
VL53L1X sensors[4];
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

// State machines for left and right sides
enum SideState { STATE_OUTSIDE, STATE_FAR, STATE_MID, STATE_NEAR };
SideState leftState = STATE_OUTSIDE;
SideState rightState = STATE_OUTSIDE;

// For single‑file detection
unsigned long lastEventTime = 0;
int lastDirection = 0;       // 1=entry, -1=exit
const unsigned long SINGLE_FILE_WINDOW = 500; // ms

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
  
  Serial.println("6‑zone people counter ready");
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
  // Reset all sensors
  for (int i = 0; i < 4; i++) {
    pinMode(xshutPins[i], OUTPUT);
    digitalWrite(xshutPins[i], LOW);
  }
  delay(10);
  
  // Enable and init one by one, assign unique addresses
  for (int i = 0; i < 4; i++) {
    pinMode(xshutPins[i], INPUT);
    delay(50);
    sensors[i].setTimeout(500);
    if (!sensors[i].init()) {
      Serial.print("Sensor ");
      Serial.print(i);
      Serial.println(" init failed");
      while (1);
    }
    sensors[i].setAddress(0x2A + i);   // 0x2A, 0x2B, 0x2C, 0x2D
    sensors[i].startContinuous(100);   // 10 Hz
    Serial.print("Sensor ");
    Serial.print(i);
    Serial.print(" at address 0x");
    Serial.println(0x2A + i, HEX);
  }
}

void readAllDistances(uint16_t dist[4]) {
  for (int i = 0; i < 4; i++) {
    dist[i] = sensors[i].read();
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
void updateCounting(Zone zone) {
  // Debounce: ignore rapid changes within 100 ms
  static Zone lastZone = ZONE_NONE;
  if (zone == lastZone) {
    lastZone = zone;
    return;
  }
  if (millis() - lastZoneChange < DEBOUNCE_MS) return;
  lastZoneChange = millis();
  lastZone = zone;
  
  // Extract side and level from zone
  int side = 0;   // 0=none, 1=left, 2=right
  int level = 0;  // 0=none, 1=far, 2=mid, 3=near
  switch (zone) {
    case LEFT_FAR:  side=1; level=1; break;
    case LEFT_MID:  side=1; level=2; break;
    case LEFT_NEAR: side=1; level=3; break;
    case RIGHT_FAR: side=2; level=1; break;
    case RIGHT_MID: side=2; level=2; break;
    case RIGHT_NEAR:side=2; level=3; break;
    default: side=0; level=0;
  }
  
  if (side == 0) {
    // No zone – reset states after 1 second of inactivity
    static unsigned long lastActive = millis();
    if (millis() - lastActive > 1000) {
      leftState = STATE_OUTSIDE;
      rightState = STATE_OUTSIDE;
    }
    return;
  }
  
  // Update last activity timestamp
  static unsigned long lastActive = millis();
  lastActive = millis();
  
  SideState *state = (side == 1) ? &leftState : &rightState;
  
  // State transition logic
  if (*state == STATE_OUTSIDE) {
    if (level == 1) *state = STATE_FAR;
    else if (level == 2) *state = STATE_MID;
    else if (level == 3) *state = STATE_NEAR; // person appears very close (started inside)
  }
  else if (*state == STATE_FAR) {
    if (level == 2) *state = STATE_MID;
    else if (level == 3) {
      *state = STATE_NEAR;
      // Reached near from far → entry event
      countEvent(1, estimateGroupSize());   // 1 = entry
    }
    else if (level == 1) { /* still far, do nothing */ }
  }
  else if (*state == STATE_MID) {
    if (level == 3) {
      *state = STATE_NEAR;
      countEvent(1, estimateGroupSize());   // entry
    }
    else if (level == 1) {
      *state = STATE_FAR;   // moving back out
    }
  }
  else if (*state == STATE_NEAR) {
    if (level == 2) *state = STATE_MID;
    else if (level == 1) {
      *state = STATE_FAR;
      countEvent(-1, estimateGroupSize());  // exit
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