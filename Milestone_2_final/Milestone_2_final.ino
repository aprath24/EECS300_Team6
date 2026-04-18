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
//   Entry: outer triggers first, then inner confirms  → +1
//   Exit:  inner triggers first, then outer confirms  → -1
enum DoorState {
  DOOR_IDLE,          // Nobody detected
  DOOR_OUTER_FIRST,   // Outer row triggered first → potential entry
  DOOR_INNER_FIRST,   // Inner row triggered first → potential exit
  DOOR_BOTH_ACTIVE    // Both rows active, waiting for resolution
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

// Debounce: passed debounce flags
unsigned long outerDebounceStart = 0;
unsigned long innerDebounceStart = 0;
bool outerConfirmed = false;
bool innerConfirmed = false;

// State machine timing
unsigned long stateEntryTime  = 0;   // when we entered current non-IDLE state
unsigned long lastActiveTime  = 0;   // last time any beam was active

// Direction tracking: set when entering BOTH_ACTIVE
bool outerWasFirst = false;

// Cooldown: prevent double-counts for the same person
unsigned long lastEventTime = 0;
int lastDirection = 0;   // 1=entry, -1=exit

// ========== Function Prototypes ==========
void initSensors();
void readAllDistances(uint16_t dist[4]);
bool isOuterRowActive(uint16_t dist[4]);
bool isInnerRowActive(uint16_t dist[4]);
void updateDebouncedRows(bool outerRaw, bool innerRaw);
void updateCounting(bool outerRaw, bool innerRaw, uint16_t dist[4]);
void countEvent(int direction);
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

  // 2. Load Non-Volatile Data CORRECTLY
  nonVol.begin("nonVolData", false);
  peopleCount = nonVol.getUInt("count", 0); // Get saved count, default to 0
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
    // Clamp out-of-range / garbage readings (e.g. 65535 from misaligned sensors)
    if (dist[i] > DIST_MAX_CAP) dist[i] = DIST_MAX_CAP;
    delay(1);
  }
}

// ========== Row Detection ==========
// A row is "active" when at least one *working* sensor on that row detects a person
bool isOuterRowActive(uint16_t dist[4]) {
  return (sensorOK[LO] && dist[LO] < DETECT_THRESH && dist[LO] > 50) ||
         (sensorOK[RO] && dist[RO] < DETECT_THRESH && dist[RO] > 50);
}

bool isInnerRowActive(uint16_t dist[4]) {
  return (sensorOK[LI] && dist[LI] < DETECT_THRESH && dist[LI] > 50) ||
         (sensorOK[RI] && dist[RI] < DETECT_THRESH && dist[RI] > 50);
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
//
// A count is only registered when the person crosses BOTH beams,
// and the first-triggered beam clears first (confirming forward motion).
// If the second-triggered beam clears first, the person turned back → no count.
//
// Handled cases:
//   • Normal walk in / out (any speed)
//   • Fast walk-through (both beams trigger nearly simultaneously)
//   • Partial entry then reversal → no count
//   • Partial exit then reversal  → no count
//   • Off-center walk (only one sensor per row triggers)
//   • Tailgating: re-arms when one beam clears while other stays blocked
void updateCounting(bool outerRaw, bool innerRaw, uint16_t dist[4]) {
  updateDebouncedRows(outerRaw, innerRaw);

  bool outerActive = outerConfirmed;
  bool innerActive = innerConfirmed;
  bool anyActive   = outerActive || innerActive;
  unsigned long now = millis();

  // Track last time anything was active
  if (anyActive) lastActiveTime = now;

  // Inactivity reset: if nothing detected for a while, go back to IDLE.
  // Exception: DOOR_BOTH_ACTIVE must continue processing even if both
  // beams just cleared (both-clear-at-once case).
  if (!anyActive && doorState != DOOR_BOTH_ACTIVE) {
    if (now - lastActiveTime > INACTIVITY_TIMEOUT) {
      if (doorState != DOOR_IDLE) {
        Serial.printf("[SM:%s] Inactivity timeout -> IDLE\n", doorStateStr(doorState));
      }
      doorState = DOOR_IDLE;
    }
    return;
  }

  switch (doorState) {

    // ── IDLE: waiting for someone to break a beam ──────────────────────
    case DOOR_IDLE:
      if (outerActive && !innerActive) {
        doorState = DOOR_OUTER_FIRST;
        stateEntryTime = now;
        Serial.printf("[SM:%s] Outer first -> potential ENTRY\n", doorStateStr(doorState));

      } else if (innerActive && !outerActive) {
        doorState = DOOR_INNER_FIRST;
        stateEntryTime = now;
        Serial.printf("[SM:%s] Inner first -> potential EXIT\n", doorStateStr(doorState));

      } else if (outerActive && innerActive) {
        // Fast walk: both triggered in same loop cycle.
        // We can't know order, default to entry (outer-first).
        doorState = DOOR_BOTH_ACTIVE;
        outerWasFirst = true;
        stateEntryTime = now;
        Serial.printf("[SM:%s] Both beams at once -> assume ENTRY\n", doorStateStr(doorState));
      }
      break;

    // ── OUTER_FIRST: outer beam broken, waiting for inner ──────────────
    case DOOR_OUTER_FIRST:
      if (innerActive) {
        // Second beam confirmed → enter BOTH_ACTIVE
        doorState = DOOR_BOTH_ACTIVE;
        outerWasFirst = true;
        Serial.printf("[SM:%s] Inner now active -> BOTH_ACTIVE\n", doorStateStr(doorState));

      } else if (!outerActive) {
        // Outer cleared without inner ever triggering → partial, no count
        Serial.printf("[SM:%s] Outer cleared alone -> partial, no count\n", doorStateStr(doorState));
        doorState = DOOR_IDLE;

      } else if (now - stateEntryTime > PARTIAL_TIMEOUT) {
        // Loitering in front of outer beam too long
        Serial.printf("[SM:%s] Partial entry timeout -> IDLE\n", doorStateStr(doorState));
        doorState = DOOR_IDLE;
      }
      break;

    // ── INNER_FIRST: inner beam broken, waiting for outer ──────────────
    case DOOR_INNER_FIRST:
      if (outerActive) {
        // Second beam confirmed → enter BOTH_ACTIVE
        doorState = DOOR_BOTH_ACTIVE;
        outerWasFirst = false;
        Serial.printf("[SM:%s] Outer now active -> BOTH_ACTIVE\n", doorStateStr(doorState));

      } else if (!innerActive) {
        // Inner cleared without outer ever triggering → partial, no count
        Serial.printf("[SM:%s] Inner cleared alone -> partial, no count\n", doorStateStr(doorState));
        doorState = DOOR_IDLE;

      } else if (now - stateEntryTime > PARTIAL_TIMEOUT) {
        // Loitering in front of inner beam too long
        Serial.printf("[SM:%s] Partial exit timeout -> IDLE\n", doorStateStr(doorState));
        doorState = DOOR_IDLE;
      }
      break;

    // ── BOTH_ACTIVE: both beams broken, waiting for resolution ─────────
    case DOOR_BOTH_ACTIVE: {

      if (!outerActive && !innerActive) {
        // ── Both cleared at once ──
        // Count based on which beam triggered first
        int dir = outerWasFirst ? 1 : -1;
        Serial.printf("[SM:%s] Both cleared -> %s\n", doorStateStr(doorState),
                      dir == 1 ? "ENTRY" : "EXIT");
        countEvent(dir);
        doorState = DOOR_IDLE;

      } else if (!outerActive && innerActive) {
        // ── Outer cleared first ──
        if (outerWasFirst) {
          // Outer triggered first AND cleared first → person moved through → ENTRY
          Serial.printf("[SM:%s] Outer cleared first (was first) -> ENTRY\n", doorStateStr(doorState));
          countEvent(1);
        } else {
          // Inner triggered first, but outer (second) cleared first → turned back
          Serial.printf("[SM:%s] Outer cleared first (was second) -> turned back, NO COUNT\n", doorStateStr(doorState));
        }
        // Re-arm: inner still active → potential new exit
        doorState = DOOR_INNER_FIRST;
        stateEntryTime = now;
        Serial.printf("[SM:%s] Re-armed: inner still active\n", doorStateStr(doorState));

      } else if (!innerActive && outerActive) {
        // ── Inner cleared first ──
        if (!outerWasFirst) {
          // Inner triggered first AND cleared first → person moved through → EXIT
          Serial.printf("[SM:%s] Inner cleared first (was first) -> EXIT\n", doorStateStr(doorState));
          countEvent(-1);
        } else {
          // Outer triggered first, but inner (second) cleared first → turned back
          Serial.printf("[SM:%s] Inner cleared first (was second) -> turned back, NO COUNT\n", doorStateStr(doorState));
        }
        // Re-arm: outer still active → potential new entry
        doorState = DOOR_OUTER_FIRST;
        stateEntryTime = now;
        Serial.printf("[SM:%s] Re-armed: outer still active\n", doorStateStr(doorState));
      }
      // else: both still active → keep waiting
      break;
    }
  }
}

// Record an event (direction: +1 entry, -1 exit)
void countEvent(int direction) {
  unsigned long now = millis();

  // Cooldown guard: suppress duplicate same-direction events that fire
  // faster than COOLDOWN_MS (same person still being tracked)
  if (lastDirection == direction && (now - lastEventTime) < COOLDOWN_MS) {
    Serial.printf("[COUNT] Suppressed duplicate %s within cooldown\n",
                  (direction == 1 ? "entry" : "exit"));
    return;
  }

  peopleCount += direction;
  if (peopleCount < 0) peopleCount = 0;

  Serial.printf("[COUNT] %s  |  Total inside: %d\n",
                (direction == 1 ? "ENTRY +1" : "EXIT -1"), peopleCount);

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
        // Serial.print("[WiFi] Received: ");
        // Serial.println(line);
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