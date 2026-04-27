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
Adafruit_VL53L1X sensorsL1[4];
Adafruit_VL53L0X sensorsL0[4];
bool useL1X = true;
bool sensorOK[4] = {false, false, false, false};
int  numSensorsOK = 0;

// ========== Global Variables ==========
volatile int32_t peopleCount = 0;
Preferences nonVol;

// ========== Per-Side State Machine ==========
enum DoorState {
  DOOR_IDLE,          // Nobody detected
  DOOR_OUTER_FIRST,   // Outer sensor triggered first → potential entry
  DOOR_INNER_FIRST,   // Inner sensor triggered first → potential exit
  DOOR_BOTH_ACTIVE,   // Both sensors active, waiting for resolution
  DOOR_WAIT_CLEAR     // Count just fired — wait for both to clear before re-arming
};

const char* doorStateStr(uint8_t s) {
  switch (s) {
    case DOOR_IDLE:        return "IDLE";
    case DOOR_OUTER_FIRST: return "OUTER_FIRST";
    case DOOR_INNER_FIRST: return "INNER_FIRST";
    case DOOR_BOTH_ACTIVE: return "BOTH_ACTIVE";
    case DOOR_WAIT_CLEAR:  return "WAIT_CLEAR";
    default:               return "???";
  }
}

struct SideSM {
  DoorState state;
  unsigned long stateEntryTime;
  unsigned long lastActiveTime;
  unsigned long outerDebStart;
  unsigned long innerDebStart;
  bool outerConf;
  bool innerConf;
  bool outerWasFirst;
};

SideSM leftSM  = {};   // zero-initialized → DOOR_IDLE, all timers 0
SideSM rightSM = {};

// Cooldown: prevent double-counts for the same person
unsigned long lastEventTime = 0;
int lastDirection = 0;   // 1=entry, -1=exit

// ========== Scenario Tracking (for client OLED debug) ==========
enum Scenario {
  SC_IDLE,             // Nothing happening
  SC_SINGLE_ENTRY,     // One person entering
  SC_SINGLE_EXIT,      // One person exiting
  SC_SHOULDER_SAME,    // Two people same direction
  SC_SHOULDER_OPP,     // Two people opposite directions
  SC_WHEELCHAIR        // Wide object (both sides triggered together)
};

volatile Scenario currentScenario = SC_IDLE;

const char* scenarioStr(uint8_t s) {
  switch (s) {
    case SC_IDLE:           return "IDLE";
    case SC_SINGLE_ENTRY:   return "ENTRY";
    case SC_SINGLE_EXIT:    return "EXIT";
    case SC_SHOULDER_SAME:  return "SH_SAME";
    case SC_SHOULDER_OPP:   return "SH_OPP";
    case SC_WHEELCHAIR:     return "WHEELCHAIR";
    default:                return "???";
  }
}

// ========== Function Prototypes ==========
void initSensors();
void readAllDistances(uint16_t dist[4]);
int  updateSide(SideSM &sm, bool outerRaw, bool innerRaw, const char* side);
void updateCounting(uint16_t dist[4]);
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

  updateCounting(dist);
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
    if (sensorOK[i]) continue;

    pinMode(xshutPins[i], INPUT);
    delay(50);

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
      dist[i] = 2000;
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

// ========== Per-Sensor Detection ==========
bool isSensorActive(int idx, uint16_t dist[4]) {
  return sensorOK[idx] && dist[idx] < DETECT_THRESH;
}

// ========== Per-Side State Machine ==========
int updateSide(SideSM &sm, bool outerRaw, bool innerRaw, const char* side) {
  unsigned long now = millis();

  // --- Per-side debounce ---
  if (outerRaw) {
    if (sm.outerDebStart == 0) sm.outerDebStart = now;
    if (now - sm.outerDebStart >= DEBOUNCE_MS) sm.outerConf = true;
  } else {
    sm.outerDebStart = 0;
    sm.outerConf = false;
  }
  if (innerRaw) {
    if (sm.innerDebStart == 0) sm.innerDebStart = now;
    if (now - sm.innerDebStart >= DEBOUNCE_MS) sm.innerConf = true;
  } else {
    sm.innerDebStart = 0;
    sm.innerConf = false;
  }

  bool outerActive = sm.outerConf;
  bool innerActive = sm.innerConf;
  bool anyActive   = outerActive || innerActive;

  if (anyActive) sm.lastActiveTime = now;

  int pendingEvent = 0;

  // --- Inactivity / WAIT_CLEAR handling ---
  if (!anyActive && sm.state != DOOR_BOTH_ACTIVE) {
    if (sm.state == DOOR_WAIT_CLEAR) {
      Serial.printf("[%s:%s] All clear -> IDLE\n", side, doorStateStr(sm.state));
      sm.state = DOOR_IDLE;
      return 0;
    }
    if (now - sm.lastActiveTime > INACTIVITY_TIMEOUT) {
      if (sm.state != DOOR_IDLE) {
        Serial.printf("[%s:%s] Inactivity -> IDLE\n", side, doorStateStr(sm.state));
      }
      sm.state = DOOR_IDLE;
    }
    return 0;
  }

  // --- State machine ---
  switch (sm.state) {

    case DOOR_IDLE:
      if (outerActive && !innerActive) {
        sm.state = DOOR_OUTER_FIRST;
        sm.stateEntryTime = now;
        Serial.printf("[%s] Outer first -> potential ENTRY\n", side);
      } else if (innerActive && !outerActive) {
        sm.state = DOOR_INNER_FIRST;
        sm.stateEntryTime = now;
        Serial.printf("[%s] Inner first -> potential EXIT\n", side);
      } else if (outerActive && innerActive) {
        sm.state = DOOR_BOTH_ACTIVE;
        sm.outerWasFirst = true;
        sm.stateEntryTime = now;
        Serial.printf("[%s] Both at once -> assume ENTRY\n", side);
      }
      break;

    case DOOR_OUTER_FIRST:
      if (innerActive) {
        sm.state = DOOR_BOTH_ACTIVE;
        sm.outerWasFirst = true;
        Serial.printf("[%s] Inner now active -> BOTH_ACTIVE\n", side);
      } else if (!outerActive) {
        Serial.printf("[%s] Outer cleared alone -> partial\n", side);
        sm.state = DOOR_IDLE;
      } else if (now - sm.stateEntryTime > PARTIAL_TIMEOUT) {
        Serial.printf("[%s] Partial timeout -> IDLE\n", side);
        sm.state = DOOR_IDLE;
      }
      break;

    case DOOR_INNER_FIRST:
      if (outerActive) {
        sm.state = DOOR_BOTH_ACTIVE;
        sm.outerWasFirst = false;
        Serial.printf("[%s] Outer now active -> BOTH_ACTIVE\n", side);
      } else if (!innerActive) {
        Serial.printf("[%s] Inner cleared alone -> partial\n", side);
        sm.state = DOOR_IDLE;
      } else if (now - sm.stateEntryTime > PARTIAL_TIMEOUT) {
        Serial.printf("[%s] Partial timeout -> IDLE\n", side);
        sm.state = DOOR_IDLE;
      }
      break;

    case DOOR_BOTH_ACTIVE:
      if (!outerActive && !innerActive) {
        pendingEvent = sm.outerWasFirst ? 1 : -1;
        Serial.printf("[%s] Both cleared -> %s\n", side,
                      pendingEvent == 1 ? "ENTRY" : "EXIT");
        sm.state = DOOR_IDLE;

      } else if (!outerActive && innerActive) {
        if (sm.outerWasFirst) {
          pendingEvent = 1;
          Serial.printf("[%s] Outer cleared first (was first) -> ENTRY\n", side);
        } else {
          Serial.printf("[%s] Outer cleared first (was second) -> turned back\n", side);
        }
        sm.state = DOOR_WAIT_CLEAR;
        sm.stateEntryTime = now;

      } else if (!innerActive && outerActive) {
        if (!sm.outerWasFirst) {
          pendingEvent = -1;
          Serial.printf("[%s] Inner cleared first (was first) -> EXIT\n", side);
        } else {
          Serial.printf("[%s] Inner cleared first (was second) -> turned back\n", side);
        }
        sm.state = DOOR_WAIT_CLEAR;
        sm.stateEntryTime = now;
      }
      // else: both still active → keep waiting
      break;

    case DOOR_WAIT_CLEAR: {
      if (!outerActive && !innerActive) {
        Serial.printf("[%s] WAIT_CLEAR -> all clear -> IDLE\n", side);
        sm.state = DOOR_IDLE;
      } else if (now - sm.stateEntryTime > PARTIAL_TIMEOUT) {
        Serial.printf("[%s] WAIT_CLEAR timeout -> IDLE\n", side);
        sm.state = DOOR_IDLE;
      }
      break;
    }
  }

  return pendingEvent;
}

// ========== Counting Orchestrator ==========
// Runs both per-side state machines and merges their results.
void updateCounting(uint16_t dist[4]) {
  // Per-sensor raw activation
  bool loRaw = isSensorActive(LO, dist);
  bool liRaw = isSensorActive(LI, dist);
  bool roRaw = isSensorActive(RO, dist);
  bool riRaw = isSensorActive(RI, dist);

  // Run both side state machines independently
  int leftDir  = updateSide(leftSM,  loRaw, liRaw, "L");
  int rightDir = updateSide(rightSM, roRaw, riRaw, "R");

  // Apply count events from both sides.
  int netCount = leftDir + rightDir;

  // --- Update scenario for client OLED debug ---
  if (netCount == 0 && leftDir == 0 && rightDir == 0) {
    // No events fired — derive scenario from current SM states
    bool lActive = (leftSM.state != DOOR_IDLE);
    bool rActive = (rightSM.state != DOOR_IDLE);

    if (!lActive && !rActive) {
      currentScenario = SC_IDLE;
    } else if (lActive && rActive) {
      // Both sides tracking — could be shoulder-by-shoulder
      currentScenario = SC_SHOULDER_SAME;
    } else {
      // One side tracking — single person
      DoorState activeState = lActive ? leftSM.state : rightSM.state;
      if (activeState == DOOR_OUTER_FIRST || 
          (activeState == DOOR_BOTH_ACTIVE && (lActive ? leftSM.outerWasFirst : rightSM.outerWasFirst))) {
        currentScenario = SC_SINGLE_ENTRY;
      } else if (activeState == DOOR_INNER_FIRST ||
                 (activeState == DOOR_BOTH_ACTIVE && !(lActive ? leftSM.outerWasFirst : rightSM.outerWasFirst))) {
        currentScenario = SC_SINGLE_EXIT;
      } else if (activeState == DOOR_WAIT_CLEAR) {
        // Keep previous scenario (just counted, waiting for clear)
      }
    }
  } else if (leftDir != 0 && rightDir != 0) {
    // Both sides fired this tick
    if (leftDir == rightDir) {
      currentScenario = SC_SHOULDER_SAME;
    } else {
      currentScenario = SC_SHOULDER_OPP;
    }
  } else if (netCount == 2 || netCount == -2) {
    currentScenario = SC_WHEELCHAIR;
  } else if (netCount > 0) {
    // Check if both sides are active (near range) for shoulder detection
    bool leftNear  = (dist[LO] < NEAR_THRESH || dist[LI] < NEAR_THRESH);
    bool rightNear = (dist[RO] < NEAR_THRESH || dist[RI] < NEAR_THRESH);
    currentScenario = (leftNear && rightNear) ? SC_SHOULDER_SAME : SC_SINGLE_ENTRY;
  } else if (netCount < 0) {
    bool leftNear  = (dist[LO] < NEAR_THRESH || dist[LI] < NEAR_THRESH);
    bool rightNear = (dist[RO] < NEAR_THRESH || dist[RI] < NEAR_THRESH);
    currentScenario = (leftNear && rightNear) ? SC_SHOULDER_SAME : SC_SINGLE_EXIT;
  } else {
    // netCount == 0 but one side fired (opposite directions on different ticks)
    // Check cross-diagonal pattern
    bool crossA = (dist[RO] <= NEAR_THRESH && dist[LI] <= NEAR_THRESH &&
                   dist[RI] <= DETECT_THRESH && dist[LO] <= DETECT_THRESH);
    bool crossB = (dist[LO] <= NEAR_THRESH && dist[RI] <= NEAR_THRESH &&
                   dist[LI] <= DETECT_THRESH && dist[RO] <= DETECT_THRESH);
    if (crossA || crossB) {
      currentScenario = SC_SHOULDER_OPP;
    }
  }
  // --- Shoulder-by-shoulder counting logic ---

  bool crossA = (dist[RO] <= NEAR_THRESH && dist[LI] <= NEAR_THRESH &&
                 dist[RI] <= DETECT_THRESH && dist[LO] <= DETECT_THRESH);
  bool crossB = (dist[LO] <= NEAR_THRESH && dist[RI] <= NEAR_THRESH &&
                 dist[LI] <= DETECT_THRESH && dist[RO] <= DETECT_THRESH);
  bool crossDetected = crossA || crossB;

  // Track when each cross pattern was last seen
  static unsigned long lastCrossATime = 0;
  static unsigned long lastCrossBTime = 0;
  static bool crossSequenceUsed = false;
  unsigned long now = millis();

  if (crossA) lastCrossATime = now;
  if (crossB) lastCrossBTime = now;

  bool crossSequence = false;
  if (crossA && lastCrossBTime > 0 && (now - lastCrossBTime) < PARTIAL_TIMEOUT && !crossSequenceUsed) {
    crossSequence = true;
  }
  if (crossB && lastCrossATime > 0 && (now - lastCrossATime) < PARTIAL_TIMEOUT && !crossSequenceUsed) {
    crossSequence = true;
  }

  // Opposite directions: confirmed cross sequence AND both sides fired (net=0).
  if (crossSequence && leftDir != 0 && rightDir != 0 && netCount == 0) {
    Serial.printf("[COUNT] Opposite shoulder-by-shoulder (cross sequence): L=%+d R=%+d\n", leftDir, rightDir);
    countEvent(leftDir);
    countEvent(rightDir);
    crossSequenceUsed = true;  // don't re-trigger for this crossing
    return;
  }

  // Reset cross sequence tracking when both sides go idle
  if (leftSM.state == DOOR_IDLE && rightSM.state == DOOR_IDLE) {
    crossSequenceUsed = false;
    lastCrossATime = 0;
    lastCrossBTime = 0;
  }

  if (!crossDetected && (netCount == 1 || netCount == -1)) {
    bool leftNear  = (dist[LO] < NEAR_THRESH || dist[LI] < NEAR_THRESH);
    bool rightNear = (dist[RO] < NEAR_THRESH || dist[RI] < NEAR_THRESH);

    if (leftNear && rightNear) {
      int sign = (netCount > 0) ? 1 : -1;
      netCount = 2 * sign;

      if (leftDir == 0) {
        Serial.printf("[L] Shoulder-by-shoulder -> skip, forced WAIT_CLEAR\n");
        leftSM.state = DOOR_WAIT_CLEAR;
        leftSM.stateEntryTime = millis();
      }
      if (rightDir == 0) {
        Serial.printf("[R] Shoulder-by-shoulder -> skip, forced WAIT_CLEAR\n");
        rightSM.state = DOOR_WAIT_CLEAR;
        rightSM.stateEntryTime = millis();
      }

      Serial.printf("[COUNT] Shoulder-by-shoulder detected -> boosted to %+d\n", netCount);
    }
  }

  if (netCount != 0) countEvent(netCount);
}

// Record an event (direction: +1/+2 entry, -1/-2 exit)
void countEvent(int direction) {
  unsigned long now = millis();
  int sign = (direction > 0) ? 1 : -1;

  if (lastDirection == sign && (now - lastEventTime) < COOLDOWN_MS) {
    Serial.printf("[COUNT] Suppressed duplicate %s within cooldown\n",
                  (sign == 1 ? "entry" : "exit"));
    return;
  }

  peopleCount += direction;
  if (peopleCount < 0) peopleCount = 0;

  Serial.printf("[COUNT] %+d  |  Total inside: %d\n", direction, peopleCount);

  lastDirection = sign;
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
        break;
    }

    // Always respond with: count, sensors, door states, scenario
    // Format: #<count>,<LI>,<LO>,<RI>,<RO>,<leftState>,<rightState>,<scenario>\n
    String response = "#" + String(peopleCount) + ","
                    + String(dist[LI]) + "," + String(dist[LO]) + ","
                    + String(dist[RI]) + "," + String(dist[RO]) + ","
                    + String((int)leftSM.state) + "," + String((int)rightSM.state) + ","
                    + String((int)currentScenario) + "\n";
    client.print(response);
    client.stop();
  }
}