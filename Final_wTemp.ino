#include <Wire.h>
#include <VL53L1X.h>
#include <Adafruit_MLX90614.h>
#include "WirelessCommunication.h"
#include "sharedVariable.h"
#include "Preferences.h"

#define BUTTON_PIN 0//boot button

Adafruit_MLX90614 mlx = Adafruit_MLX90614();

// The number of sensors in your system.
const uint8_t sensorCount = 2;

// The Arduino pin connected to the XSHUT pin of each sensor.
const uint8_t xshutPins[sensorCount] = {18, 16};

VL53L1X sensors[sensorCount];

// Variables to manage detection timing and state.
long sensor1DetectTime = 0;
long sensor2DetectTime = 0;
bool sensor1Detected = false;
bool sensor2Detected = false;

bool counted = false;
bool backed = false;
bool alrRead = false;

int og = 0;

int people = 1;

uint16_t og1dist = 1000;
uint16_t og2dist = 1000;

float base = 0;
float diff = 1;
float temp = 0;
bool first = true;
//float temp_thresh = 71.5;

void init_non_vol_storage();
void update_non_vol_count();
void update_button_count();

volatile int32_t count = 0;
volatile shared_uint32 x;
Preferences nonVol;//used to store the count in nonvolatile memory

// Threshold distance to detect an object (in mm).
//was at 800
//change these numbers, was 600 before 4/20, 700 at end of 4/20
const int detectionThreshold = 750;
//TODO test this, was 100, i upped to 200 last night (on 4/20)
const int twoPeopleThreshold = 200;

void TOF_sensors(){ //------------------------------------- not complete
  // Read distance from each sensor.
  uint16_t distanceSensor1 = sensors[0].read();
  uint16_t distanceSensor2 = sensors[1].read();
  //is tripped
  if(distanceSensor1 < detectionThreshold || distanceSensor2 < detectionThreshold){
    if (og == 0) {
      //finding which sensor was tripped first
      //one tripped, not two
      if(distanceSensor1 < detectionThreshold && distanceSensor2 >= detectionThreshold){
        Serial.println("One Tripped");
        og1dist = distanceSensor1;
        og = 1;
      }
      //two tripped, not one
      else if (distanceSensor2 < detectionThreshold && distanceSensor1 >= detectionThreshold){
        Serial.println("Two Tripped");
        og2dist = distanceSensor2;
        og = 2;
      }
      else if (distanceSensor1 < detectionThreshold && distanceSensor2 < detectionThreshold) {
        Serial.println("Both Tripped 1");
        og1dist = distanceSensor1;
        og2dist = distanceSensor2;
        og = 3;

        //whlie both are tripped
        while (distanceSensor1 < detectionThreshold && distanceSensor2 < detectionThreshold) {
          distanceSensor1 = sensors[0].read();
          distanceSensor2 = sensors[1].read();
        }

        //if only 1 is tripped
        if (distanceSensor1 < detectionThreshold) {
          while (distanceSensor1 < detectionThreshold) {
            distanceSensor1 = sensors[0].read();
            distanceSensor2 = sensors[1].read();
          }
          //both --> one --> none = OUT
          //TODO do i need to add a check for temp here?
          count = count - people;
          counted = true;
          if(count < 0) count = 0;
          Serial.print("Both --> Out, Count: ");
          Serial.println(count);
          og = 0;
        }

        //if only 2 is tripped
        else if (distanceSensor2 < detectionThreshold) {
          while (distanceSensor1 < detectionThreshold) {
            distanceSensor1 = sensors[0].read();
            distanceSensor2 = sensors[1].read();
          }
          //both --> two --> none = IN
          //todo do i need to check for temp here?
          count = count + people;
          counted = true;
          Serial.print("Both --> IN, Count: ");
          Serial.println(count);
          og = 0;
        }

        //if none are tripped
        else {
          //todo do i need to check for temp here
          Serial.print("Both --> None, guessing IN, Count: ");
          count = count + people;
          counted = true;
          Serial.println(count);
          og = 0;
        }
        //TODO needs a special case, while/until smth is tripped, should take care of cascade in here
      }
      //none tripped, shouldn't happen
      else if (distanceSensor1 > detectionThreshold && distanceSensor2 > detectionThreshold) {
        //Serial.print("Sensor 1: "); Serial.print(distanceSensor1); Serial.print("  Sensor 2: "); Serial.println(distanceSensor2);
        Serial.println("None Tripped");
      }
      //this should never happen!
      else {
        Serial.print("Sensor 1: "); Serial.print(distanceSensor1); Serial.print("  Sensor 2: "); Serial.println(distanceSensor2);
        Serial.println("Other");

      }
    }

    //second sensor type tripped
    else {
      while (!counted && !backed) {
        //one tripped, not two
        if(distanceSensor1 < detectionThreshold && distanceSensor2 >= detectionThreshold){
          //Serial.println("One Tripped");
          if (og == 1) {
            //do nothing, nothing is new
          }
          else if (og == 2){
            Serial.println("One Tripped");

            if (og1dist == 1000) {
              og1dist = distanceSensor1;
            }

            if (first) {  //if first person to go through tof
              base = mlx.readObjectTempF();
              Serial.print("Base: "); Serial.println(base);
              first = false;
            }
            temp = mlx.readObjectTempF();
            //OUT
            while (distanceSensor1 < detectionThreshold) {
              distanceSensor1 = sensors[0].read();
              distanceSensor2 = sensors[1].read();
            }
            //distanceSensor1 not tripped
            if (distanceSensor2 < detectionThreshold) {
              //went back to distanceSensor2, nothing changes
            }
            //both are not tripped = 2 --> 1 --> OUT
            else {
              // ----------------------------------------------------------------------------------------------------------------------------
              if((temp > (base + diff))) {
                Serial.print("Temp: "); Serial.println(temp);
                Serial.print("Temp High Enough: OG Dist 1: "); Serial.print(og1dist); Serial.print("  OG Dist 2: "); Serial.println(og2dist);
                
                if ((og1dist < twoPeopleThreshold || og2dist < twoPeopleThreshold)) {
                  people = 2;
                }
                //Serial.print("Temp: ") + Serial.println(temp);
              }
              Serial.print("People:  ");
              Serial.println(people);

              count = count - people;
              counted = true;
              if(count < 0) count = 0;
              Serial.print("Out, Count: ");
              Serial.println(count);
              Serial.println("  ");
              og = 0;
            }
          }
        }
        //two tripped, not one
        else if (distanceSensor2 < detectionThreshold && distanceSensor1 >= detectionThreshold){
          // Serial.println("Two Tripped");
          if (og == 2) {
            //do nothing, nothing is new
          }
          //hedging bets that too fast equal in?
          else if (og == 1 || og == 3){
            Serial.println("Two Tripped");

            if (og2dist == 1000) {
              og2dist = distanceSensor2;
            }

            if (first) {  //if first person to go through tof
              base = mlx.readObjectTempF();
              Serial.print("Base: "); Serial.println(base);
              first = false;
            }
            temp = mlx.readObjectTempF();

            //IN
            while (distanceSensor2 < detectionThreshold) {
              distanceSensor1 = sensors[0].read();
              distanceSensor2 = sensors[1].read();
            }
            //distanceSensor2 not tripped
            if (distanceSensor1 < detectionThreshold) {
              //went back to distanceSensor1, nothing changes
            }
            //both are not tripped = 1 --> 2 --> IN
            else {
              if((temp > (base + diff))) {
                Serial.print("Temp: "); Serial.println(temp);
                Serial.print("Temp High Enough: OG Dist 1: "); Serial.print(og1dist); Serial.print("  OG Dist 2: "); Serial.println(og2dist);

                if ((og1dist < twoPeopleThreshold || og2dist < twoPeopleThreshold)) {
                  people = 2;
                }
                //Serial.print("Temp: ") + Serial.println(temp);
              }
              Serial.print("People:  ");
              Serial.println(people);

              count = count + people;
              counted = true;
              if(count < 0) count = 0;
              if (og == 3) {
                Serial.print("Fake In, Count: ");
              }
              else {
                Serial.print("In, Count: ");
              }
              Serial.println(count);
              Serial.println("  ");
              og = 0;
            }
          }
        }
        else if (distanceSensor1 < detectionThreshold && distanceSensor2 < detectionThreshold) {
          Serial.println("Both Tripped 2");
          if (og1dist == 1000) {
            og1dist = distanceSensor1;
          }
          if (og2dist == 1000) {
            og2dist = distanceSensor2;
          }

          if (first) {  //if first person to go through tof
            base = mlx.readObjectTempF();
            Serial.print("Base: "); Serial.println(base);
            first = false;
          }
          temp = mlx.readObjectTempF();

          while (distanceSensor1 < detectionThreshold && distanceSensor2 < detectionThreshold){
            distanceSensor1 = sensors[0].read();
            distanceSensor2 = sensors[1].read();
          }

          //only one is tripped
          if (distanceSensor1 < detectionThreshold) {
            //do nothing?
            alrRead = true;
          }
          //only two is tripped
          else if (distanceSensor2 < detectionThreshold) {
            //do nothing?
            alrRead = true;
          }
          //neither are tripped!
          //TODO maybe an else if statement instead
          else {
            
            //if og has a value, and then went to both and then none, no one is turning around that fast
            //so i can safely?? count that based on which was og
            //og 1 --> both --> none = IN
            if (og == 1) {
              if((temp > (base + diff))) {
                Serial.print("Temp: "); Serial.println(temp);
                Serial.print("Temp High Enough: OG Dist 1: "); Serial.print(og1dist); Serial.print("  OG Dist 2: "); Serial.println(og2dist);

                if ((og1dist < twoPeopleThreshold || og2dist < twoPeopleThreshold)) {
                  people = 2;
                }
                //Serial.print("Temp: ") + Serial.println(temp);
              }
              Serial.print("People:  ");
              Serial.println(people);

              count = count + people;
              counted = true;
              Serial.print("Both In, Count: ");
              Serial.println(count);
              Serial.println("  ");
              og = 0;
            }
            //og 2 --> both --> none = OUT
            else if (og == 2) {
              if((temp > (base + diff))) {
                Serial.print("Temp: "); Serial.println(temp);
                Serial.print("Temp High Enough: OG Dist 1: "); Serial.print(og1dist); Serial.print("  OG Dist 2: "); Serial.println(og2dist);

                if ((og1dist < twoPeopleThreshold || og2dist < twoPeopleThreshold)) {
                  people = 2;
                }
                //Serial.print("Temp: ") + Serial.println(temp);
              }
              Serial.print("People:  ");
              Serial.println(people);

              count = count - people;
              counted = true;
              if(count < 0) count = 0;
              Serial.print("Both Out, Count: ");
              Serial.println(count);
              Serial.println("  ");
              og = 0;
            }
            else {
              Serial.println("idk what else it could be");
            }
          }
          //Serial.print("Sensor 1: "); Serial.print(distanceSensor1); Serial.print("  Sensor 2: "); Serial.println(distanceSensor2);
        }
        //none tripped so backed out!
        else if (distanceSensor1 > detectionThreshold && distanceSensor2 > detectionThreshold) {
          Serial.print("None Tripped = Backed out: ");
          Serial.println(og);
          backed = true;
          og = 0;
        }
        //this should never happen!
        else {
          Serial.print("Sensor 1: "); Serial.print(distanceSensor1); Serial.print("  Sensor 2: "); Serial.println(distanceSensor2);
          Serial.println("uhoh Other");

        }

        if (!alrRead) {
          distanceSensor1 = sensors[0].read();
          distanceSensor2 = sensors[1].read();
        }

        alrRead = false;
      }
    }
  }
  counted = false;
  backed = false;
  people = 1;
}

void setup() {
  Serial.begin(115200);
  Wire.begin();

   if (!mlx.begin()) {
    Serial.println("Error connecting to MLX sensor. Check wiring.");
    while (1);
  };

  Serial.print("Emissivity = "); Serial.println(mlx.readEmissivity());
  Serial.println("================================================");


  // Disable/reset all sensors by driving their XSHUT pins low.
  for (uint8_t i = 0; i < sensorCount; i++) {
    pinMode(xshutPins[i], OUTPUT);
    digitalWrite(xshutPins[i], LOW);
  }

  delay(10); // Ensure a reset happens

  // Enable, initialize, and start each sensor, one by one.

  for (uint8_t i = 0; i < sensorCount; i++) {
    pinMode(xshutPins[i], INPUT); // Stop driving XSHUT low
    delay(50); // Wait for the sensor to start up

    sensors[i].setTimeout(500);
    if (!sensors[i].init()) {
      Serial.print("Failed to detect and initialize sensor ");
      Serial.println(i);
      while (1);
    }

    sensors[i].setAddress(0x2A + i); // Unique I2C address

    sensors[i].startContinuous(50);
  }

  Serial.println("End Setup");

  init_wifi_task();
  init_non_vol_count();//initializes nonvolatile memory and retrieves latest count
  count = 0;
  INIT_SHARED_VARIABLE(x, count);//init shared variable used to tranfer info to WiFi core

}



void loop() {

  TOF_sensors();

  update_button_count();//update shared variable x (shared with WiFi task)
  update_non_vol_count();//updates nonvolatile count 

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

//initializes nonvolatile memory and retrieves latest count
void init_non_vol_count()
{
  nonVol.begin("nonVolData", false);//Create a “storage space” in the flash memory called "nonVolData" in read/write mode
  count = nonVol.getUInt("count", 0);//attempts to retrieve "count" from nonVolData, sets it 0 if not found
}

//updates nonvolatile memery with lates value of count
void update_non_vol_count()
{
  if(count < 0) count = 0;
  nonVol.putUInt("count", count);//write count to nonvolatile memory
}

//example code that updates a shared variable (which is printed to server)
//under the hood, this implementation uses a semaphore to arbitrate access to x.value
void update_button_count()
{
  //minimized time spend holding semaphore
  LOCK_SHARED_VARIABLE(x);
  if(count < 0) count = 0;
  x.value = count;
  UNLOCK_SHARED_VARIABLE(x);   
}
