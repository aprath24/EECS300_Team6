#include "WirelessCommunication.h"

//WiFi name and password, change the name to match your team, and choose your own password
#define STASSID "team6"
#define STAPSK "66666666"

//uncomment following line to enable the debug outpus associated with wifi stuff over serial
//#define ENABLE_SERIAL_DEBUG_OUTPUTS

static TaskHandle_t task_loop1;

static const char* ssid = STASSID;
static const char* password = STAPSK;

static const char* host = "192.168.4.1";//observed to be default IP of server ESP
static const uint16_t port = 80;
static WiFiMulti multi;

//used to share data between cores
extern volatile shared_uint32 x;
extern volatile shared_uint32 distPack0;
extern volatile shared_uint32 distPack1;
extern volatile bool serverConnected;
extern volatile bool resetRequested;

//like setup() and loop(), but run on the other core

void setup1()
{
  wireless_init();//init WiFi hardware and connect to network
}

//reads the people count + ToF data from the server
void loop1()
{
  WiFiClient client;
  connect_to_server(client);

  // If a reset was requested, send #0 to zero the server count
  if (resetRequested) {
    write_to_server(client, "#0\n");
    resetRequested = false;
  } else {
    write_to_server(client, "read\n");
  }

  // Read the server's response (format: #count,LI,LO,RI,RO\n)
  String response = read_from_server(client);
  handle_reboot_request(client);
  client.stop();

  if (response.length() > 0 && response[0] == '#') {
    // Strip the '#'
    String data = response.substring(1);

    // Parse comma-separated values
    int idx1 = data.indexOf(',');
    if (idx1 > 0) {
      uint32_t serverCount = data.substring(0, idx1).toInt();
      LOCK_SHARED_VARIABLE(x);
      x.value = serverCount;
      UNLOCK_SHARED_VARIABLE(x);
      serverConnected = true;

      // Parse the 4 distances
      String rest_str = data.substring(idx1 + 1);
      int idx2 = rest_str.indexOf(',');
      int idx3 = rest_str.indexOf(',', idx2 + 1);
      int idx4 = rest_str.indexOf(',', idx3 + 1);

      if (idx2 > 0 && idx3 > 0 && idx4 > 0) {
        uint16_t dLI = rest_str.substring(0, idx2).toInt();
        uint16_t dLO = rest_str.substring(idx2 + 1, idx3).toInt();
        uint16_t dRI = rest_str.substring(idx3 + 1, idx4).toInt();
        uint16_t dRO = rest_str.substring(idx4 + 1).toInt();

        LOCK_SHARED_VARIABLE(distPack0);
        distPack0.value = ((uint32_t)dLI << 16) | dLO;
        UNLOCK_SHARED_VARIABLE(distPack0);

        LOCK_SHARED_VARIABLE(distPack1);
        distPack1.value = ((uint32_t)dRI << 16) | dRO;
        UNLOCK_SHARED_VARIABLE(distPack1);
      }
    }
  }

  rest(200);
}

/*
 * Function:  wireless_init
 * --------------------
 * initializes and connects to WiFi
 */
static void wireless_init()
{
  #ifdef ENABLE_SERIAL_DEBUG_OUTPUTS
    Serial.begin(115200);
    Serial.println();
    Serial.println();
    Serial.print("Connecting to ");
    Serial.println(ssid);
  #endif
  // We start by connecting to a WiFi network
  multi.addAP(ssid, password);

  while (multi.run() != WL_CONNECTED)
  {
    #ifdef ENABLE_SERIAL_DEBUG_OUTPUTS
      Serial.println("Connecting");
    #endif
    rest(500);
  }
  #ifdef ENABLE_SERIAL_DEBUG_OUTPUTS
    Serial.print("WiFi connection successful with IP ");
    Serial.println(WiFi.localIP());
  #endif

  WiFiClient client;
  connect_to_server(client);    
  write_to_server(client, "client started\n");
  client.stop();  
}

static void connect_to_server(WiFiClient &client)
{
  while (!client.connect(host, 80)) 
  {
    #ifdef ENABLE_SERIAL_DEBUG_OUTPUTS
      Serial.println("Connection to server failed");
    #endif

    if (WiFi.status() != WL_CONNECTED)
    {
      #ifdef ENABLE_SERIAL_DEBUG_OUTPUTS
        Serial.print("WiFi disconnected");
      #endif
      while (multi.run() != WL_CONNECTED)
      {
        #ifdef ENABLE_SERIAL_DEBUG_OUTPUTS
          Serial.println("Reconnecting");
        #endif
        rest(500);
      }
      write_to_server(client, "client started\n");
      #ifdef ENABLE_SERIAL_DEBUG_OUTPUTS
        Serial.print("WiFi reconnection successful with IP ");
        Serial.println(WiFi.localIP());
      #endif
    }
    rest(10);
  }
}

static void write_to_server(WiFiClient &client, String value)
{
  client.print(value);
  #ifdef ENABLE_SERIAL_DEBUG_OUTPUTS
    Serial.print("Sending: ");
    if(value.endsWith("\n")) Serial.print(value);
    else Serial.println(value);
  #endif
}

static void update_count(uint32_t count)
{
  WiFiClient client;
  String transmitString = "#" + String(count) + "\n";
  connect_to_server(client);    
  write_to_server(client, transmitString);;    
  handle_reboot_request(client);
  client.stop();  
}

static String read_from_server(WiFiClient &client)
{
  client.setTimeout(2);
  //wait and see if we get a line from the server
  return client.readStringUntil('\n');
}

static void handle_reboot_request(WiFiClient &client)
{
  if(read_from_server(client)[0] == 'r')
  {
    client.stop();
    #ifdef ENABLE_SERIAL_DEBUG_OUTPUTS
      Serial.println("Rebooting");
    #endif
      
  ESP.restart();
  }
}

static void esploop1(void* pvParameters)
{
  setup1();
  for (;;)
    loop1();
}

void init_wifi_task()
{
  xTaskCreatePinnedToCore(
    esploop1,               /* Task function. */
    "loop1",                /* name of task. */
    10000,                  /* Stack size of task */
    NULL,                   /* parameter of the task */
    0,                      /* priority of the task */
    &task_loop1,            /* Task handle to keep track of created task */
    !ARDUINO_RUNNING_CORE); /* pin task to core */  
}

void rest(uint16_t delay_ms)
{
  vTaskDelay(delay_ms / portTICK_PERIOD_MS);
}
