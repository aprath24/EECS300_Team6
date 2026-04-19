/*
 * Client board for people counter system.
 * 
 * - Polls the server for the current people count and displays it on the OLED.
 * - Shows debug info: sensor distances and door state machine states.
 * - Pressing the boot button sends a reset command (#0) to the server
 *   to set the count back to zero.
 * 
 * The server (with ToF sensors) is the source of truth for the count.
 */
#include "WirelessCommunication.h"
#include "sharedVariable.h"

#define BUTTON_PIN 0  // boot button

uint32_t is_pressed();

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// OLED Configuration
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET    -1
#define SCREEN_ADDRESS 0x3C

// Hardware Instances
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// Shared variables (between main core and WiFi core)
volatile shared_server_data serverData;  // full server response (WiFi -> main)
volatile shared_uint32 reset_flag;       // reset request: main core writes 1, WiFi core reads & clears

// Convert door state enum value to readable string
const char* doorStateStr(uint8_t s) {
  switch (s) {
    case 0:  return "IDLE";
    case 1:  return "OUTER_1ST";
    case 2:  return "INNER_1ST";
    case 3:  return "BOTH";
    case 4:  return "WAIT_CLR";
    default: return "???";
  }
}

void setup()
{
  pinMode(BUTTON_PIN, INPUT);
  Serial.begin(115200);
  
  // Initialize I2C (OLED only)
  // SDA = 21, SCL = 22
  Wire.begin(21, 22);

  if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("OLED allocation failed"));
    while(1);
  }

  // Initialize shared variables
  serverData.count = 0;
  serverData.distLI = 0; serverData.distLO = 0;
  serverData.distRI = 0; serverData.distRO = 0;
  serverData.leftState = 0; serverData.rightState = 0;
  serverData.sem = xSemaphoreCreateMutex();

  INIT_SHARED_VARIABLE(reset_flag, 0);
  init_wifi_task();
  
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0,0);
  display.println("Client Ready");
  display.println("Connecting to Team 6...");
  display.display();
}

void loop()
{           
  // Button press -> request a count reset to 0
  if(is_pressed())
  {
    LOCK_SHARED_VARIABLE(reset_flag);
    reset_flag.value = 1;
    UNLOCK_SHARED_VARIABLE(reset_flag);
    Serial.println("[BTN] Reset requested");
  }

  // Read the latest server data from the WiFi core
  uint32_t count;
  uint16_t dLI, dLO, dRI, dRO;
  uint8_t  lState, rState;

  LOCK_SHARED_VARIABLE(serverData);
  count  = serverData.count;
  dLI    = serverData.distLI;
  dLO    = serverData.distLO;
  dRI    = serverData.distRI;
  dRO    = serverData.distRO;
  lState = serverData.leftState;
  rState = serverData.rightState;
  UNLOCK_SHARED_VARIABLE(serverData);

  // Update the OLED with debug info
  update_oled(count, dLI, dLO, dRI, dRO, lState, rState);
  
  // Moderate refresh rate
  delay(100);
}


void update_oled(uint32_t count, uint16_t dLI, uint16_t dLO,
                 uint16_t dRI, uint16_t dRO,
                 uint8_t lState, uint8_t rState) {
  display.clearDisplay();
  
  // Row 0: Header + count (large)
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.print("COUNT:");
  display.setTextSize(2);
  display.setCursor(50, 0);
  display.print(count);

  // Row 2: Separator
  display.drawLine(0, 18, 128, 18, SSD1306_WHITE);

  // Row 3: Sensor distances
  display.setTextSize(1);
  display.setCursor(0, 22);
  display.printf("LI:%4u LO:%4u", dLI, dLO);
  display.setCursor(0, 32);
  display.printf("RI:%4u RO:%4u", dRI, dRO);

  // Row 5: Separator
  display.drawLine(0, 42, 128, 42, SSD1306_WHITE);

  // Row 6-7: Door states
  display.setCursor(0, 46);
  display.printf("L: %s", doorStateStr(lState));
  display.setCursor(0, 56);
  display.printf("R: %s", doorStateStr(rState));
  
  display.display();
}

uint32_t is_pressed()
{
  if(!digitalRead(BUTTON_PIN))
  {
    delay(10);//software debouncing
    if(!digitalRead(BUTTON_PIN))
    {
      while(!digitalRead(BUTTON_PIN));//make sure button is depressed
      return 1;
    }
  }
  return 0;
}
