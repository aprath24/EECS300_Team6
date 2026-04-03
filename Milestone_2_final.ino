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
int people = 1;

void init_non_vol_storage();
void update_non_vol_count();
void update_button_count();

volatile int32_t count = 0;
volatile shared_uint32 x;
Preferences nonVol;//used to store the count in nonvolatile memory

// Threshold distance to detect an object (in mm).
const int detectionThreshold = 800; // Example threshold; adjust based on your setup.

void TOF_sensors(){ //------------------------------------- not complete
  // Read distance from each sensor.
  uint16_t distanceSensor1 = sensors[0].read();
  uint16_t distanceSensor2 = sensors[1].read();

  while(1) Serial.println(mlx.readObjectTempF());

  //Serial.println("TOF");

  //long currentTime = millis();

  if(distanceSensor1 < detectionThreshold || distanceSensor2 < detectionThreshold){ // check if either sensor is tripped
  //Serial.println("detected");
  


    if(mlx.readObjectTempF() > 71) people = 2;
    Serial.print("People:  ");
    Serial.println(people);

    if(distanceSensor1 < detectionThreshold && distanceSensor2 >= detectionThreshold){ // if sensor 1 is tripped first (in) ------------
      //Serial.println("Sen 1 tripped");
      //sensor1DetectTime = currentTime;
      //sensor1Detected = true;

      while(distanceSensor1 < detectionThreshold && counted == false){ // while sensor 1 stays tripped
      //Serial.println("while 1");

        if(distanceSensor2 >= detectionThreshold); // sensor 1 is tripped but not sensor 2, do nothing
        else if(distanceSensor2 < detectionThreshold){ // sensor 1 and 2 tripped
          while(distanceSensor2 < detectionThreshold){ // while sensor 2 is still tripped
            if(distanceSensor1 >= detectionThreshold){  // sensor 1 no longer tripped
              while(distanceSensor2 < detectionThreshold){ // while sensor 2 still tripped
                distanceSensor1 = sensors[0].read();
                distanceSensor2 = sensors[1].read();
              }
              if(distanceSensor1 >= detectionThreshold){ // sensor 1 and 2 no longer tripped
                count = count + people;
                counted = true;
                Serial.println("In");
                Serial.println(count);
              }
              
            } 

            distanceSensor1 = sensors[0].read();
            distanceSensor2 = sensors[1].read();
          }
          // sensor2DetectTime = currentTime;
          // sensor2Detected = true;
          
        }
        distanceSensor1 = sensors[0].read();
        distanceSensor2 = sensors[1].read();

      }
      while(distanceSensor1 < detectionThreshold) distanceSensor1 = sensors[0].read();
    }

    else if(distanceSensor2 < detectionThreshold && distanceSensor1 >= detectionThreshold){ // sensor 2 tripped first (out) -------------------
      //sensor2DetectTime = currentTime;
      //sensor2Detected = true;


      while(distanceSensor2 < detectionThreshold && counted == false){ // while sensor 2 stays tripped
      //Serial.println("while 1");

        if(distanceSensor1 >= detectionThreshold); // sensor 2 is tripped but not sensor 1, do nothing
        else if(distanceSensor1 < detectionThreshold){ // sensor 1 and 2 tripped
          while(distanceSensor1 < detectionThreshold){ // while sensor 1 is still tripped
            if(distanceSensor2 >= detectionThreshold){  // sensor 2 no longer tripped
              while(distanceSensor1 < detectionThreshold){ // while sensor 1 still tripped
                distanceSensor1 = sensors[0].read();
                distanceSensor2 = sensors[1].read();
              }
              if(distanceSensor2 >= detectionThreshold){ // sensor 1 and 2 no longer tripped
                count = count - people;
                counted = true;
                if(count < 0) count = 0;
                Serial.println("Out");
                Serial.println(count);
              }
              
            } 

            distanceSensor1 = sensors[0].read();
            distanceSensor2 = sensors[1].read();
          }
          // sensor2DetectTime = currentTime;
          // sensor2Detected = true;
          
        }
        distanceSensor1 = sensors[0].read();
        distanceSensor2 = sensors[1].read();

      }

      // while(distanceSensor2 < detectionThreshold && counted == false){ // while sensor 2 stays tripped
      // //Serial.println("while 2");
      //   if(distanceSensor1 >= detectionThreshold);
      //   else if(distanceSensor1 < detectionThreshold){
      //     // sensor1DetectTime = currentTime;
      //     // sensor1Detected = true;
      //     count = count - people;
      //     counted = true;
      //     if(count < 0) count = 0;
      //     Serial.println("Out");
      //     Serial.println(count);
      //   }
      //   distanceSensor1 = sensors[0].read();
      //   distanceSensor2 = sensors[1].read();
      // }
      while(distanceSensor2 < detectionThreshold) distanceSensor2 = sensors[1].read();
    }

    else if(distanceSensor1 < detectionThreshold && distanceSensor2 < detectionThreshold){
      if(distanceSensor1 < distanceSensor2){
        count = count + people;
        counted = true;
        Serial.println("In");
        Serial.println(count);
    }
      else{
        count = count - people;
        counted = true;
        if(count < 0) count = 0;
        Serial.println("Out");
        Serial.println(count);
      }
      while(distanceSensor1 < detectionThreshold && distanceSensor2 < detectionThreshold){
        distanceSensor1 = sensors[0].read();
        distanceSensor2 = sensors[1].read();
      }
    }
  }
  counted = false;
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
  
  Serial.println(mlx.readObjectTempF());

  TOF_sensors();

  update_button_count();//update shared variable x (shared with WiFi task)
  update_non_vol_count();//updates nonvolatile count 
  
  // Serial.print(sensors[0].read());
  // Serial.print("    ");
  // Serial.println(sensors[1].read());

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

