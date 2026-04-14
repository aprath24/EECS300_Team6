#include "WirelessCommunication.h"

//WiFi name and password, change the name to match your team, and choose your own password
#define STASSID "team0000"
#define STAPSK "testPassword"

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

//like setup() and loop(), but run on the other core

void setup1()
{
  wireless_init();//init WiFi hardware and connect to network
}

//example code, this will update the server as fast as it can. You can add a rest(x) call to delay by x ms
void loop1()
{
  static uint32_t shared_val = 0;
  //minimized time spend holding semaphore
  LOCK_SHARED_VARIABLE(x);
  shared_val = x.value;
  UNLOCK_SHARED_VARIABLE(x);
  
  update_count(shared_val);
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
