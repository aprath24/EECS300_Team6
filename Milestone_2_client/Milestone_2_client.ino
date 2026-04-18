/*
 * Client board for people counter system.
 * 
 * - Polls the server for the current people count and displays it on the OLED.
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
volatile shared_uint32 x;           // server count: WiFi core writes, main core reads
volatile shared_uint32 reset_flag;  // reset request: main core writes 1, WiFi core reads & clears

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

  INIT_SHARED_VARIABLE(x, 0);
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

  // Read the latest server count from the WiFi core
  uint32_t count;
  LOCK_SHARED_VARIABLE(x);
  count = x.value;
  UNLOCK_SHARED_VARIABLE(x);

  // Serial.println("the count is: " + String(count));

  // Update the OLED with the current people count
  update_oled(count);
  
  // Moderate refresh rate
  delay(100);
}


void update_oled(uint32_t count) {
  display.clearDisplay();
  
  // Header
  display.setTextSize(1);
  display.setCursor(0,0);
  display.println("PEOPLE COUNTER");
  display.drawLine(0, 12, 128, 12, SSD1306_WHITE);

  // Count Label
  display.setCursor(0, 20);
  display.println("Total Inside:");

  // Large Count Display
  display.setTextSize(4);
  display.setCursor(40, 32);
  display.print(count);
  
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
