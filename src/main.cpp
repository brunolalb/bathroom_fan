/* 
Smart Fan Controller for my bathroom's exhaust fan

Bruno Landau Albrecht
brunolalb@gmail.com

Use at your own will and responsibility

ESP32 is connected to
* PIR Sensor
* DHT11 Humidity and Temperature Sensor
* RGB LED
* Relay

The Code is FreeRTOS based
*/


#include <stdio.h>
// Configuration
#include "HardwareConfig.h"
// FreeRTOS - official FreeRTOS lib
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/timers.h"
#include "freertos/event_groups.h"
// WiFi, NTP, WiFi management, and OTA
#include "WiFiHandler.h"
#include "RGBLed.h"
#include "DHTSensor.h"
#include "PIRSensor.h"
#include "SwitchSensor.h"
#include "ConfigManager.h"

// debug stuff
SemaphoreHandle_t semph_debug; // controls access to the debug stuff
//#define EXAUSTOR_TEST

/****************************************
 * Forward Declarations
 ****************************************/
void debug(const char *msg);
void debug_nonFreeRTOS(const char *msg);
void control_loop(void *params);

/* WiFi Handler */
WiFiHandler wifiHandler(WIFI_HOSTNAME, WIFI_AP_NAME);

/* Configuration Manager */
ConfigManager configManager;
DeviceConfig deviceConfig;

/* RGB LED */
RGBLed led(LED_PIN_RED, LED_PIN_GREEN, LED_PIN_BLUE);

/* DHT Sensor */
DHTSensor dhtSensor(DHTPIN, DHTTYPE);

/* PIR Sensor */
PIRSensor pirSensor(PIR_PIN);

/* Switch Sensor */
SwitchSensor switchSensor(SWITCH_PIN, SWITCH_DEBOUNCE_TIME_MS);

// Global for NTP time access via WiFiHandler
#define timeClient (wifiHandler.getTimeClient())



/* Global variables */
long relay_on_time = 0;    // Controls how long relay stays ON
long relay_keep_off = 15;  // Controls how long relay stays OFF

SemaphoreHandle_t semph_relay; // controls access to the relay_on_time and relay_keep_off

/****************************************
 * Debug
 ****************************************/

void debug(const char *msg) 
{
  if (xPortInIsrContext()) {
    Serial.print("ISR!!");
    Serial.println(msg);
    return;
  }

  if(xSemaphoreTake(semph_debug, pdMS_TO_TICKS(100)) == pdTRUE ) {
    Serial.println(msg);
    xSemaphoreGive(semph_debug);    
  }  
}

void debug_nonFreeRTOS(const char *msg)
{
  Serial.println(msg);
}

void setup_debug()
{
  semph_debug = xSemaphoreCreateMutex();
  xSemaphoreGive(semph_debug);
}



/****************************************
 * Control Loop
 ****************************************/
void setup_control()
{
  /* Set the Relay pin to output */
  pinMode(RELAY_PIN, OUTPUT);
  RELAY_OFF();

  xTaskCreate(control_loop,
          "Control",   // A name just for humans
          (uint32_t)20000,  // Stack size
          NULL,            // parameters
          (UBaseType_t)2,  // priority
          NULL );          // task pointer
}

void control_loop(void *params)
{
  TickType_t xLastWakeTime = xTaskGetTickCount();
  char msg[50];
  bool humidity_high = false;  
  DHTData dht_data;
  QueueHandle_t dhtQueue = dhtSensor.getQueue();

  while (1) {
    // humidity check
    dht_data.humidity = -1.0;
    dht_data.temperature = -273.0;
    if ((dhtQueue) && (xQueueReceive(dhtQueue, &dht_data, pdMS_TO_TICKS(100)) == pdPASS)) {
      if (!isnan(dht_data.humidity) && (dht_data.humidity >= 0)) {
        // check if humidity within ranges
        if (dht_data.humidity >= deviceConfig.humidity_limit_high) { // high enough to turn the relay on
          humidity_high = true;
          if (!led.isBlueOn()) {
            snprintf(msg, 50, "humidity high: %.1f %%", dht_data.humidity);
            debug(msg);
          }
          led.blueOn();
        } else if (dht_data.humidity <= deviceConfig.humidity_limit_low) { // low enough to stop controlling
          humidity_high = false;        
          if (led.isBlueOn()) {
            snprintf(msg, 50, "humidity low: %.1f %%", dht_data.humidity);
            debug(msg);
          }
          led.blueOff();
        } else {
          //between humidity_limit_high and humidity_limit_low
        }

        if (humidity_high) {
          if( xSemaphoreTake(semph_relay, pdMS_TO_TICKS(100)) == pdTRUE ) {
            if ((relay_keep_off == 0) && (relay_on_time < RELAY_ON_TIME_SEC_MIN)) { // the relay is not on (or will not be on for long)
              relay_on_time = RELAY_ON_TIME_SEC_HUMIDITY; // turn it on for 30 secs at least
            }
            xSemaphoreGive(semph_relay);
          } else {
            debug("control_loop - humidity: failed to take mutex relay");            
          }
        } else {
          // humidity is low
          // if it was high, then just let the timer run out
        }
      } else {
        // humidity wasn't correctly read
      }
    } else {      
      // nothing on the queue
    }

    // movement detection check
    if (pirSensor.isMovementDetected() && timeClient.isTimeSet()) {
      pirSensor.resetMovement();

      bool ignore = false;

      // Calculate if current time is between ignore intervals (quiet hours)
      int now = timeClient.getHours() * 60 + timeClient.getMinutes();
      int quietStart = deviceConfig.quiet_time_start_hour * 60 + deviceConfig.quiet_time_start_min;
      int quietEnd = deviceConfig.quiet_time_end_hour * 60 + deviceConfig.quiet_time_end_min;

      if (quietStart < quietEnd) {
        // Interval does not cross midnight
        if (now >= quietStart && now < quietEnd) ignore = true;
      } else {
        // Interval crosses midnight
        if (now >= quietStart || now < quietEnd) ignore = true;
      }

      if (ignore) {
        debug("Movement ignored due to time interval");
      } else {
        debug("Movement detected");
        if(xSemaphoreTake(semph_relay, pdMS_TO_TICKS(100)) == pdTRUE ) {
          if (relay_keep_off == 0) { // the relay is not off
            relay_on_time = deviceConfig.pir_relay_on_time;
          }
          xSemaphoreGive(semph_relay);
        } else {
          debug("control_loop - movement: failed to take mutex");
        }
      }
    }

    // switch check
#if SWITCH_ENABLE == 1    
    if (switchSensor.isSwitched()) {
      switchSensor.resetSwitch();
      
      debug("Switch Changed");
      if(xSemaphoreTake(semph_relay, pdMS_TO_TICKS(100)) == pdTRUE ) {    
        if (relay_on_time) {
          // was on, turn off and remain off for some time
          relay_on_time = 0;
          relay_keep_off = RELAY_KEEP_OFF_TIME_SEC_DEFAULT;
        } else {
          // was off, turn on for the default time
          relay_on_time = RELAY_ON_TIME_SEC_DEFAULT; 
          relay_keep_off = 0;
        }   
        xSemaphoreGive(semph_relay);
      } else {
        debug("control_loop - switch: failed to take mutex");
      }
    } 
#endif

    // relay control
    if( xSemaphoreTake(semph_relay, pdMS_TO_TICKS(500)) == pdTRUE ) {
      if (relay_keep_off > 0) {
        if (relay_keep_off % 10 == 0) {
          snprintf(msg, 50, "Relay keep off: %lu", relay_keep_off);
          debug(msg);
        }
        relay_keep_off--;
        RELAY_OFF();
        led.greenOff();        
      } else if (relay_on_time > 0) {
        if (relay_on_time % 10 == 0) {
          snprintf(msg, 50, "Relay on: %lu", relay_on_time);
          debug(msg);
        }
        relay_on_time--;
        if (!RELAY_IS_ON()) {
          debug("Relay ON");
        }
        RELAY_ON();
        led.greenOn();
      } else {
        if (RELAY_IS_ON()) {
          relay_keep_off = 5;
          debug("IDLE");
        }          
        RELAY_OFF();
        led.greenOff();
      }
      xSemaphoreGive(semph_relay);
    } else {
      debug("control_loop: failed to take mutex");
    }

    vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(CONTROL_LOOP_PERIOD_MS));
  }

}

/****************************************
 * Application Setup
 ****************************************/

void setup() 
{
  Serial.begin(115200);

  // CRITICAL: Long delay to ensure ESP32 hardware is fully initialized
  // This includes WiFi hardware, lwIP stack, and FreeRTOS core
  // Without this, lwIP is not ready when we try to use WiFi
  delay(3000);
  
  debug_nonFreeRTOS("=== Starting bathroom fan controller ===");

  // Initialize LED pins early to show status
  led.begin();

  setup_debug();
  
  /* Configuration Manager - Initialize and load settings */
  debug_nonFreeRTOS("Initializing configuration...");
  if (!configManager.begin()) {
    debug_nonFreeRTOS("ERROR: Failed to initialize ConfigManager!");
  }
  if (!configManager.loadConfig(deviceConfig)) {
    debug_nonFreeRTOS("Using default configuration");
  }
  
  /* WiFi Setup - FIRST after delay to ensure lwIP is ready */
  debug_nonFreeRTOS("Initializing WiFi...");
  wifiHandler.begin(&led);

  /* OTA Update stuff */
  wifiHandler.setupOTA();

  /* Configuration Web Server */
  wifiHandler.setupConfigWebServer(&configManager, &deviceConfig);

  // semaphore for the relay_on_time
  semph_relay = xSemaphoreCreateMutex();
  if (semph_relay == NULL) {
    debug_nonFreeRTOS("Could not create the Relay Mutex!");
  }
  xSemaphoreGive(semph_relay);

  /* control loop */
  setup_control();

  /* setup the switch */
#if SWITCH_ENABLE == 1
  switchSensor.begin();
#endif
  
  /* start DHT sensor */
  dhtSensor.begin();  

  /* setup movement sensor */
  pirSensor.begin();

  debug_nonFreeRTOS("Setup complete - all systems initialized");
}

/****************************************
 * Application Loop
 ****************************************/
void loop() 
{
  wifiHandler.loop();  // Handles WiFiManager and OTA updates
}
