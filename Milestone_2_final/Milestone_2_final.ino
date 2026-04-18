#include <Wire.h>
#include "Adafruit_VL53L1X.h"
#include "Adafruit_VL53L0X.h"
#include "Preferences.h"
#include "config.h"
#include <WiFi.h>
#include <esp_task_wdt.h>


WiFiServer server(80);

const char *ssid = "team6";  // TODO: Fill in with team number, must match in client sketch
const char *password = "66666666";  // At least 8 chars, must match in client sketch


esp_task_wdt_config_t watchdog = {.timeout_ms = 5000, .trigger_panic = true}; 


// ========== Object Declarations ==========
Adafruit_VL53L1X sensorsL1[4];   // Adafruit VL53L1X sensor objects
Adafruit_VL53L0X sensorsL0[4];   // Adafruit VL53L0X sensor objects (fallback)
bool useL1X = true;      // true = using L1X, false = using L0X (set during init)
bool sensorOK[4] = {false, false, false, false};  // which sensors initialized
int  numSensorsOK = 0;   // count of successfully initialized sensors

// ========== Global Variables ==========
volatile int32_t peopleCount = 0;
Preferences nonVol;

// Dual-beam state machine for direction detection
//   Outer row = LO, RO  (hallway side of door frame)
//   Inner row = LI, RI  (room side of door frame)
//   Entry: outer triggers first, then inner confirms
//   Exit:  inner triggers first, then outer confirms
enum DoorState {
  DOOR_IDLE,          // Nobody detected
  DOOR_OUTER_FIRST,   // Outer row triggered first → potential entry
  DOOR_INNER_FIRST,    // Inner row triggered first → potential exit
  DOOR_BOTH_ACTIVE
};
DoorState doorState = DOOR_IDLE;

const char* doorStateStr(DoorState s) {
  switch (s) {
    case DOOR_IDLE:        return "IDLE";
    case DOOR_OUTER_FIRST: return "OUTER_FIRST";
    case DOOR_INNER_FIRST: return "INNER_FIRST";
    case DOOR_BOTH_ACTIVE: return "BOTH_ACTIVE";
    default:               return "???";
  }
}
bool eventFired = false;      // has an entry/exit been counted this traversal?

// Timestamps for when each row first became active in this cycle
unsigned long outerFirstActiveTime = 0;
unsigned long innerFirstActiveTime = 0;
bool outerWasActive = false;  // previous-loop state for edge detection
bool innerWasActive = false;

// Debounce: passed debounce flags
unsigned long outerDebounceStart = 0;
unsigned long innerDebounceStart = 0;
bool outerConfirmed = false;  // passed debounce
bool innerConfirmed = false;

// Partial-entry timeout: state management
unsigned long stateEntryTime = 0;  // when we entered OUTER_FIRST or INNER_FIRST

// For single-file / tailgating detection logic
unsigned long lastEventTime = 0;
int lastDirection = 0;       // 1=entry, -1=exit
int peakGroupSize = 1;

// Inactivity tracking variables
unsigned long lastActiveTime = 0;

// ========== Function Prototypes ==========
void initSensors();
void readAllDistances(uint16_t dist[4]);
bool isOuterRowActive(uint16_t dist[4]);
bool isInnerRowActive(uint16_t dist[4]);
void updateDebouncedRows(bool outerRaw, bool innerRaw);
void updateCounting(bool outerRaw, bool innerRaw, uint16_t dist[4]);
int estimateGroupSize(uint16_t dist[4]);
void countEvent(int direction, int group);
void updateNonVolatile();
void handleWiFiClient(uint16_t dist[4]);

// ========== Setup ==========
void setup() {
  Serial.begin(115200);
    
  // WiFi connection procedure
  WiFi.mode(WIFI_AP);
  WiFi.softAP(ssid, password);
  Serial.print("ESP32 IP as soft AP: ");
  Serial.println(WiFi.softAPIP());
  
  server.begin();

  //watchdog timer with 5s period
  esp_task_wdt_init(&watchdog); //enable watchdog (which will restart ESP32 if it hangs)
  esp_task_wdt_add(NULL); //add current thread to WDT watch
  
  Serial.println("server started\n");

  // Boot button for count reset
  pinMode(BUTTON_PIN, INPUT);


  
  Wire.begin();
  Wire.setClock(400000);            // 400 kHz I2C
  Wire.setTimeOut(100);             // 100ms I2C timeout (ESP32)
  initSensors();

  nonVol.begin("nonVolData", false);
  peopleCount = 0;
  nonVol.putUInt("count", 0);

  Serial.println("Layout: LO/RO (outer row) | LI/RI (inner row)");
}

// ========== Main Loop ==========
void loop() {
  uint16_t dist[4];
  readAllDistances(dist);

  // Debug: print raw sensor distances only when count changes
  {
    static int32_t lastPrintedCount = -1;
    if (peopleCount != lastPrintedCount) {
      Serial.printf("[DBG] LI:%4u  LO:%4u  RI:%4u  RO:%4u mm | Count: %d\n",
                    dist[LI], dist[LO], dist[RI], dist[RO], peopleCount);
      lastPrintedCount = peopleCount;
    }
  }

  bool outerActive = isOuterRowActive(dist);
  bool innerActive = isInnerRowActive(dist);

  updateCounting(outerActive, innerActive, dist);
  handleWiFiClient(dist);
  updateNonVolatile();

  // Boot button: press to reset count to zero
  if (!digitalRead(BUTTON_PIN)) {
    delay(50);  // debounce
    if (!digitalRead(BUTTON_PIN)) {
      while (!digitalRead(BUTTON_PIN));  // wait for release
      peopleCount = 0;
      nonVol.putUInt("count", 0);
      Serial.println("[RESET] Count reset to 0 via button");
    }
  }

  esp_task_wdt_reset();  // feed the watchdog
  delay(20);   // small delay to prevent I2C flooding
}

// ========== I2C Probe ==========
// Quick check if any device ACKs at the given address.
// Returns true if a device responded, false otherwise.
bool i2cProbe(uint8_t addr) {
  Wire.beginTransmission(addr);
  return (Wire.endTransmission() == 0);
}

// ========== Sensor Initialization ==========
void initSensors() {
  const char* sensorNames[] = {"LI", "LO", "RI", "RO"};

  // Reset all sensors via XSHUT
  for (int i = 0; i < 4; i++) {
    pinMode(xshutPins[i], OUTPUT);
    digitalWrite(xshutPins[i], LOW);
  }
  delay(10);

  // --- Auto-detect sensor type on the first available sensor ---
  bool typeDetected = false;
  for (int i = 0; i < 4 && !typeDetected; i++) {
    pinMode(xshutPins[i], INPUT);   // bring sensor i out of reset
    delay(50);

    // Quick I2C probe at default address 0x29 – skip if nothing responds
    if (!i2cProbe(0x29)) {
      Serial.printf("[INIT] Sensor %d (%s) not found on bus – skipping\n", i, sensorNames[i]);
      continue;
    }

    // Try VL53L1X first
    if (sensorsL1[i].begin(0x2A + i, &Wire)) {
      useL1X = true;
      typeDetected = true;
      sensorOK[i] = true;
      sensorsL1[i].startRanging();
      Serial.printf("[INIT] Detected VL53L1X/VL53L1CX via sensor %d (%s) at 0x%02X\n",
                    i, sensorNames[i], 0x2A + i);
    }
    // Try VL53L0X fallback
    else if (sensorsL0[i].begin(0x2A + i)) {
      useL1X = false;
      typeDetected = true;
      sensorOK[i] = true;
      Serial.printf("[INIT] Detected VL53L0X via sensor %d (%s) at 0x%02X\n",
                    i, sensorNames[i], 0x2A + i);
    }
    else {
      Serial.printf("[INIT] Sensor %d (%s) responded but init failed – skipping\n", i, sensorNames[i]);
    }
  }

  if (!typeDetected) {
    Serial.println("[INIT] FATAL: no sensors detected at all!");
    while (1);
  }

  // --- Init remaining sensors with the detected type ---
  for (int i = 0; i < 4; i++) {
    if (sensorOK[i]) continue;  // already initialized above

    pinMode(xshutPins[i], INPUT);
    delay(50);

    // Quick I2C probe – skip if nothing on the bus
    if (!i2cProbe(0x29)) {
      Serial.printf("[INIT] Sensor %d (%s) not found on bus – skipping\n", i, sensorNames[i]);
      continue;
    }

    bool ok = false;
    if (useL1X) {
      ok = sensorsL1[i].begin(0x2A + i, &Wire);
      if (ok) sensorsL1[i].startRanging();
    } else {
      ok = sensorsL0[i].begin(0x2A + i);
    }

    sensorOK[i] = ok;
    if (ok) {
      Serial.printf("[INIT] Sensor %d (%s) OK at 0x%02X\n",
                    i, sensorNames[i], 0x2A + i);
    } else {
      Serial.printf("[INIT] Sensor %d (%s) FAILED – will run without it\n",
                    i, sensorNames[i]);
    }
  }

  // Count working sensors
  numSensorsOK = 0;
  for (int i = 0; i < 4; i++) if (sensorOK[i]) numSensorsOK++;
  Serial.printf("[INIT] %d of 4 sensors online\n", numSensorsOK);

  if (numSensorsOK == 0) {
    Serial.println("[INIT] FATAL: zero sensors online!");
    while (1);
  }
}

void readAllDistances(uint16_t dist[4]) {
  for (int i = 0; i < 4; i++) {
    if (!sensorOK[i]) {
      dist[i] = 2000;  // treat missing sensor as "no detection"
      continue;
    }
    if (useL1X) {
      if (sensorsL1[i].dataReady()) {
        dist[i] = sensorsL1[i].distance();
        sensorsL1[i].clearInterrupt();
      }
    } else {
      VL53L0X_RangingMeasurementData_t measure;
      sensorsL0[i].rangingTest(&measure, false);
      if (measure.RangeStatus != 4) {
        dist[i] = measure.RangeMilliMeter;
      } else {
        dist[i] = 2000;
      }
    }
    delay(1);
  }
}

// ========== Row Detection ==========
// A row is "active" when at least one *working* sensor on that row detects a person
bool isOuterRowActive(uint16_t dist[4]) {
  return (sensorOK[LO] && dist[LO] < DETECT_THRESH) ||
         (sensorOK[RO] && dist[RO] < DETECT_THRESH);
}

bool isInnerRowActive(uint16_t dist[4]) {
  return (sensorOK[LI] && dist[LI] < DETECT_THRESH) ||
         (sensorOK[RI] && dist[RI] < DETECT_THRESH);
}

// ========== Debounced Row Detection ==========
// Returns true only if the raw row signal has been continuously active
// for at least DEBOUNCE_MS.  This filters out momentary noise / reflections
// that happen when someone brushes the door frame.
void updateDebouncedRows(bool outerRaw, bool innerRaw) {
  unsigned long now = millis();

  // --- Outer row debounce ---
  if (outerRaw) {
    if (outerDebounceStart == 0) outerDebounceStart = now;
    if (now - outerDebounceStart >= DEBOUNCE_MS) outerConfirmed = true;
  } else {
    outerDebounceStart = 0;
    outerConfirmed = false;
  }

  // --- Inner row debounce ---
  if (innerRaw) {
    if (innerDebounceStart == 0) innerDebounceStart = now;
    if (now - innerDebounceStart >= DEBOUNCE_MS) innerConfirmed = true;
  } else {
    innerDebounceStart = 0;
    innerConfirmed = false;
  }
}

// ========== Counting State Machine ==========
// Direction is determined by which row (outer vs inner) triggers first.
//   Outer first → person walking IN from hallway  → entry (+1)
//   Inner first → person walking OUT from room    → exit  (-1)
// The event is counted when the SECOND row also becomes active,
// confirming the person has crossed both beams.
//
// Edge-case handling:
//   • Tailgating: after an event fires, the machine re-arms as soon as
//     the first-triggered row clears (even if the other is still blocked
//     by the next person).
//   • Fast traversal: when both rows trigger simultaneously from IDLE,
//     timestamps are compared to infer direction.
//   • Door-frame loitering: if only one row ever triggers, the partial-
//     entry timeout resets the machine without counting.
void updateCounting(bool outerRaw, bool innerRaw, uint16_t dist[4]) {
  updateDebouncedRows(outerRaw, innerRaw);

  bool outerActive = outerConfirmed;
  bool innerActive = innerConfirmed;
  bool anyActive = outerActive || innerActive;
  unsigned long now = millis();

  if (outerActive && !outerWasActive) outerFirstActiveTime = now;
  if (innerActive && !innerWasActive) innerFirstActiveTime = now;
  outerWasActive = outerActive;
  innerWasActive = innerActive;

  // Only immediately return on inactivity if we are NOT in the middle of
  // resolving a crossing. DOOR_BOTH_ACTIVE must still be allowed to process
  // the case where both beams have just cleared.
  if (!anyActive && doorState != DOOR_BOTH_ACTIVE) {
    if (now - lastActiveTime > INACTIVITY_TIMEOUT) {
      doorState = DOOR_IDLE;
      eventFired = false;
    }
    return;
  }

  if (anyActive) {
    lastActiveTime = now;
  }

  switch (doorState) {
    case DOOR_IDLE:
      if (outerActive && !innerActive) {
        doorState = DOOR_OUTER_FIRST;
        stateEntryTime = now;
        eventFired = false;
        Serial.printf("[SM:%s] Outer first -> potential ENTRY\n", doorStateStr(doorState));
      } else if (innerActive && !outerActive) {
        doorState = DOOR_INNER_FIRST;
        stateEntryTime = now;
        eventFired = false;
        Serial.printf("[SM:%s] Inner first -> potential EXIT\n", doorStateStr(doorState));
      } else if (outerActive && innerActive) {
        doorState = (outerFirstActiveTime <= innerFirstActiveTime)
                    ? DOOR_OUTER_FIRST
                    : DOOR_INNER_FIRST;
        stateEntryTime = now;
        eventFired = false;
        Serial.printf("[SM:%s] Simultaneous -> inferred direction\n", doorStateStr(doorState));
      }
      break;

    case DOOR_OUTER_FIRST:
      if (innerActive) {
        doorState = DOOR_BOTH_ACTIVE;
        Serial.printf("[SM:%s] Both beams active -> waiting to see which clears first\n", doorStateStr(doorState));
      } else if (!eventFired && (now - stateEntryTime > PARTIAL_TIMEOUT)) {
        Serial.printf("[SM:%s] Partial entry timeout - resetting\n", doorStateStr(doorState));
        doorState = DOOR_IDLE;
        eventFired = false;
      }
      break;

    case DOOR_INNER_FIRST:
      if (outerActive) {
        doorState = DOOR_BOTH_ACTIVE;
        Serial.printf("[SM:%s] Both beams active -> waiting to see which clears first\n", doorStateStr(doorState));
      } else if (!eventFired && (now - stateEntryTime > PARTIAL_TIMEOUT)) {
        Serial.printf("[SM:%s] Partial exit timeout - resetting\n", doorStateStr(doorState));
        doorState = DOOR_IDLE;
        eventFired = false;
      }
      break;

    case DOOR_BOTH_ACTIVE: {
      // Track peak group size throughout this state (not just at moment of clearing)
      int currentGroup = estimateGroupSize(dist);
      if (currentGroup > peakGroupSize) peakGroupSize = currentGroup;

      bool outerWasFirst = (outerFirstActiveTime <= innerFirstActiveTime);
      unsigned long timeDiff = (outerFirstActiveTime > innerFirstActiveTime)
          ? outerFirstActiveTime - innerFirstActiveTime
          : innerFirstActiveTime - outerFirstActiveTime;
      bool isCrossing = (timeDiff < SIMULTANEOUS_THRESH_MS);

      if (!outerActive && !innerActive) {
        // ── Both beams cleared ──────────────────────────────────────────
        if (isCrossing) {
          Serial.printf("[SM:%s] Simultaneous crossing detected -> net 0, no count\n", doorStateStr(doorState));
          // Uncomment below if you want gross traffic stats:
          // countEvent(1, 1); countEvent(-1, 1);
        } else {
          Serial.printf("[SM:%s] Both cleared -> counting\n", doorStateStr(doorState));
          countEvent(outerWasFirst ? 1 : -1, peakGroupSize);
        }
        doorState = DOOR_IDLE;
        eventFired = false;

      } else if (!outerActive && innerActive) {
        // ── Outer cleared first ─────────────────────────────────────────
        if (outerWasFirst) {
          Serial.printf("[SM:%s] Outer cleared first (was first) -> ENTRY counted\n", doorStateStr(doorState));
          countEvent(1, peakGroupSize);
          Serial.printf("[SM:%s] Re-armed: inner still active -> potential EXIT (tailgate)\n", doorStateStr(doorState));
        } else {
          Serial.printf("[SM:%s] Outer cleared first (was second) -> turned back, NO COUNT\n", doorStateStr(doorState));
          Serial.printf("[SM:%s] Re-armed: inner still active after turn-back\n", doorStateStr(doorState));
        }
        peakGroupSize = 1;  // reset for next traversal
        doorState = DOOR_INNER_FIRST;
        stateEntryTime = now;
        eventFired = false;

      } else if (!innerActive && outerActive) {
        // ── Inner cleared first ─────────────────────────────────────────
        if (!outerWasFirst) {
          Serial.printf("[SM:%s] Inner cleared first (was first) -> EXIT counted\n", doorStateStr(doorState));
          countEvent(-1, peakGroupSize);
          Serial.printf("[SM:%s] Re-armed: outer still active -> potential ENTRY (tailgate)\n", doorStateStr(doorState));
        } else {
          Serial.printf("[SM:%s] Inner cleared first (was second) -> turned back, NO COUNT\n", doorStateStr(doorState));
          Serial.printf("[SM:%s] Re-armed: outer still active after turn-back\n", doorStateStr(doorState));
        }
        peakGroupSize = 1;  // reset for next traversal
        doorState = DOOR_OUTER_FIRST;
        stateEntryTime = now;
        eventFired = false;
      }
      // if both still active, keep waiting
      break;
    }
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

  // Cooldown guard: if the exact same direction fires again faster than
  // COOLDOWN_MS it is almost certainly the SAME person still being
  // tracked (not a new person).  Suppress the duplicate.
  if (lastDirection == direction && (now - lastEventTime) < COOLDOWN_MS) {
    Serial.printf("[COUNT] Suppressed duplicate %s within cooldown\n",
                  (direction == 1 ? "entry" : "exit"));
    return;
  }

  peopleCount += direction * group;
  if (peopleCount < 0) peopleCount = 0;

  Serial.printf("[COUNT] %s %+d  |  Total inside: %d\n",
                (direction == 1 ? "ENTRY" : "EXIT"),
                direction * group, peopleCount);

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

// ========== WiFi Server Handler ==========
// When a client connects, send the current count + ToF distances
// and check for incoming commands.
void handleWiFiClient(uint16_t dist[4]) {
  WiFiClient client = server.available();
  if (!client) return;

  client.setTimeout(2);
  if (client.connected()) {
    String line = client.readStringUntil('\n');

    switch (line[0]) {
      case '#':  // set count (from another counter board)
        peopleCount = line.substring(1).toInt();
        Serial.printf("[WiFi] Count set to: %d\n", peopleCount);
        break;
      case '+':  // increment
        ++peopleCount;
        Serial.printf("[WiFi] Incremented, count: %d\n", peopleCount);
        break;
      case '-':  // decrement
        if (peopleCount > 0) --peopleCount;
        Serial.printf("[WiFi] Decremented, count: %d\n", peopleCount);
        break;
      case '\0': // empty — just a read request
        break;
      default:
        Serial.print("[WiFi] Received: ");
        Serial.println(line);
        break;
    }

    // Always respond with the current count + sensor distances
    String response = "#" + String(peopleCount) + ","
                    + String(dist[LI]) + "," + String(dist[LO]) + ","
                    + String(dist[RI]) + "," + String(dist[RO]) + "\n";
    client.print(response);
    client.stop();
  }
}