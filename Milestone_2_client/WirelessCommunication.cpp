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
extern volatile shared_server_data serverData;  // full server response (WiFi -> main)
extern volatile shared_uint32 reset_flag;       // reset request (main -> WiFi)

//like setup() and loop(), but run on the other core

void setup1()
{
  wireless_init();//init WiFi hardware and connect to network
}

// Polls the server for the current count + debug data.
// If the main core requested a reset, sends "#0" first.
void loop1()
{
  // Check if main core requested a reset
  uint32_t do_reset = 0;
  LOCK_SHARED_VARIABLE(reset_flag);
  do_reset = reset_flag.value;
  reset_flag.value = 0;  // clear the flag
  UNLOCK_SHARED_VARIABLE(reset_flag);
  
  // Poll the server (and send reset if requested)
  poll_server(do_reset);
  
  rest(200); // poll ~5 times per second
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

static String read_from_server(WiFiClient &client)
{
  client.setTimeout(1000); // 1 second — enough time for server to respond
  //wait and see if we get a line from the server
  return client.readStringUntil('\n');
}

/*
 * Function:  poll_server
 * --------------------
 * Connects to the server, optionally sends a reset command,
 * then reads back the server's current count + debug data.
 * Parses: #<count>,<LI>,<LO>,<RI>,<RO>,<leftState>,<rightState>
 * 
 * do_reset: if non-zero, sends "#0" to reset server count to 0
 */
static void poll_server(uint32_t do_reset)
{
  WiFiClient client;
  connect_to_server(client);

  if (do_reset) {
    write_to_server(client, "#0\n");  // tell server to set count to 0
  } else {
    write_to_server(client, "\n");    // empty request = just read
  }

  // Server responds with: "#<count>,<LI>,<LO>,<RI>,<RO>,<leftState>,<rightState>\n"
  String response = read_from_server(client);
  client.stop();

  // Parse the full response into shared data
  uint32_t count = 0;
  uint16_t dLI = 0, dLO = 0, dRI = 0, dRO = 0;
  uint8_t  lState = 0, rState = 0;

  if (response.length() > 0 && response[0] == '#') {
    // Tokenize by commas: count, LI, LO, RI, RO, leftState, rightState
    int idx = 1;  // skip '#'
    int field = 0;
    while (idx <= (int)response.length() && field < 7) {
      int nextComma = response.indexOf(',', idx);
      if (nextComma < 0) nextComma = response.length();
      String token = response.substring(idx, nextComma);
      switch (field) {
        case 0: count  = token.toInt(); break;
        case 1: dLI    = token.toInt(); break;
        case 2: dLO    = token.toInt(); break;
        case 3: dRI    = token.toInt(); break;
        case 4: dRO    = token.toInt(); break;
        case 5: lState = token.toInt(); break;
        case 6: rState = token.toInt(); break;
      }
      idx = nextComma + 1;
      field++;
    }
  }

  // Pass all parsed data to main core
  LOCK_SHARED_VARIABLE(serverData);
  serverData.count   = count;
  serverData.distLI  = dLI;
  serverData.distLO  = dLO;
  serverData.distRI  = dRI;
  serverData.distRO  = dRO;
  serverData.leftState  = lState;
  serverData.rightState = rState;
  UNLOCK_SHARED_VARIABLE(serverData);
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
