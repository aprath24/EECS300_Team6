/*
 * Client that reads the people count and ToF sensor
 * distances from the Milestone 2 server (people counter)
 * and prints them for debugging.
 */
#include "WirelessCommunication.h"
#include "sharedVariable.h"

volatile uint32_t count = 0;
volatile shared_uint32 x;          // shared: people count
volatile shared_uint32 distPack0;  // shared: LI (high 16) | LO (low 16)
volatile shared_uint32 distPack1;  // shared: RI (high 16) | RO (low 16)
volatile bool serverConnected = false;  // set by WiFi core on first successful read
volatile bool resetRequested = false;   // set by main loop when button pressed

#define BUTTON_PIN 0  // boot button

void setup()
{
  Serial.begin(115200);
  pinMode(BUTTON_PIN, INPUT);
  init_wifi_task();
  INIT_SHARED_VARIABLE(x, count);
  INIT_SHARED_VARIABLE(distPack0, 0);
  INIT_SHARED_VARIABLE(distPack1, 0);
}

void loop()
{
  if (!serverConnected) {
    delay(500);
    return;
  }

  // Read the latest values from shared variables (updated by WiFi core)
  uint32_t dp0, dp1;

  LOCK_SHARED_VARIABLE(x);
  count = x.value;
  UNLOCK_SHARED_VARIABLE(x);

  LOCK_SHARED_VARIABLE(distPack0);
  dp0 = distPack0.value;
  UNLOCK_SHARED_VARIABLE(distPack0);

  LOCK_SHARED_VARIABLE(distPack1);
  dp1 = distPack1.value;
  UNLOCK_SHARED_VARIABLE(distPack1);

  // Unpack distances
  uint16_t dLI = (dp0 >> 16) & 0xFFFF;
  uint16_t dLO = dp0 & 0xFFFF;
  uint16_t dRI = (dp1 >> 16) & 0xFFFF;
  uint16_t dRO = dp1 & 0xFFFF;

  Serial.printf("[CLIENT] Count: %u  |  LI:%4u  LO:%4u  RI:%4u  RO:%4u mm\n",
                count, dLI, dLO, dRI, dRO);

  // Boot button: press to request count reset on server
  if (!digitalRead(BUTTON_PIN)) {
    delay(5);
    if (!digitalRead(BUTTON_PIN)) {
      while (!digitalRead(BUTTON_PIN));  // wait for release
      resetRequested = true;
      Serial.println("[CLIENT] Reset requested — sending to server...");
    }
  }

  delay(5);
}
