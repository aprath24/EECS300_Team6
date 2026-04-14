#ifndef WIRELESS_COMMUNICATION_H_
#define WIRELESS_COMMUNICATION_H_

#include <WiFi.h>
#include <WiFiMulti.h>
#include <stdint.h>
#include "sharedVariable.h"

/*
 * Function:  wireless_init
 * --------------------
 * initializes and connects to WiFi
 */
static void wireless_init();

/*
 * Function:  connect_to_server
 * --------------------
 * connects to server, this is blocking; code execution will stay 
 * in this function until a connection to server is made
 * 
 * also handles dropped WiFi events
 * 
 * client:  instance of WiFIClient that we want to connect
 */
static void connect_to_server(WiFiClient &client);

/*
 * Function:  read_from_server
 * --------------------
 * reads data, if availble, from server. Will wait for 2 seconds before giving up
 * 
 * client:  instance of WiFIClient (expected to be already be connected to server)
 * 
 * returns the read data (not including the '\n') returns empty string if there is no data available
 */
static String read_from_server(WiFiClient &client);

/*
 * Function:  write_to_server
 * --------------------
 * writes the provided String to the server
 * 
 * client:  instance of WiFIClient (expected to be already be connected to server)
 * value:   String to be written to server
 */
static void write_to_server(WiFiClient &client, String value);

/*
 * Function:  update_count
 * --------------------
 * updates server with provided count
 * 
 * count: value to be sent to server
 */
static void update_count(uint32_t count);

/*
 * Function:  handle_reboot_request
 * --------------------
 * checks if the server sent a reboot request and reboots if requested
 * 
 * client:  instance of WiFIClient (expected to be already be connected to server)
 */
static void handle_reboot_request(WiFiClient &client);

//attaches setup1() and loop1() to the WiFi core (core0)
static void esploop1(void* pvParameters);

/*
 * Function:  init_wifi_task
 * --------------------
 * Begins the WiFi code on core0 (the regular Arduino code runs on core1)
 */
void init_wifi_task();

/*
 * Function:  rest()\\
 * --------------------
 * idles for the requested timel; unlike delay this does not block
 * 
 * delay_ms:  delay in milliseconds.
 */
void rest(uint16_t delay_ms);
   


#endif /* INC_WIRELESS_COMMUNICATION_H_ */
