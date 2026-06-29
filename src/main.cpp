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
#ifdef EXAUSTOR_TEST
#define WIFI_HOSTNAME "exaustor_test"
#define WIFI_AP_NAME "exaustor-test-setup"
#else
#define WIFI_HOSTNAME "exaustor"
#define WIFI_AP_NAME "exaustor-setup"
#endif
WiFiHandler wifiHandler(WIFI_HOSTNAME, WIFI_AP_NAME, "admin");

/* RGB LED */
RGBLed led(2, 5, 21);  // Red pin: 2, Green pin: 5, Blue pin: 21

/* DHT Sensor */
DHTSensor dhtSensor(19, DHT11);  // DHT11 on pin 19

/* PIR Sensor */
PIRSensor pirSensor(18);  // PIR sensor on pin 18

// Global for NTP time access via WiFiHandler
#define timeClient (wifiHandler.getTimeClient())

/* OTA Update */
// OTA is now managed by WiFiHandler

/****************************************
 * Application Configuration
 ****************************************/

/* define DHT pins */
#define DHTPIN    19
#define DHTTYPE   DHT11

/* Relay */
#define RELAY_PIN     4
#define RELAY_ON()    digitalWrite(RELAY_PIN, LOW)
#define RELAY_OFF()   digitalWrite(RELAY_PIN, HIGH)
#define RELAY_IS_ON() !digitalRead(RELAY_PIN)

/* Switch */
#define USE_SWITCH      0
#define SWITCH_PIN      22
#define SWITCH_IS_ON()  digitalRead(SWITCH_PIN)
#define SWITCH_DEBOUNCE_TIME_MS   500
bool switch_switched = false;

/* Application */
#define CONTROL_LOOP_PERIOD_MS       1000 // everything happens every 1 second
#define CONTROL_PERIOD_SEC_DEFAULT   10  // 10sec
#define CONTROL_PERIOD_SEC_MIN       1   // 1sec

#define HUMIDITY_LIMIT_HIGH_DEFAULT 60.0        // anything higher will trigger the relay
#define HUMIDITY_LIMIT_LOW_DEFAULT  55.0        // anything lower than this and the relay will shutoff

#define RELAY_ON_TIME_SEC_DEFAULT       300  // if the relay was turned on by presence (or switch), it'll stay on for this much time
#define RELAY_ON_TIME_SEC_HUMIDITY      60   // if humidity is high, relay will be on for this many seconds
#define RELAY_ON_TIME_SEC_MIN           30   // minimum time the relay will be on
#define RELAY_KEEP_OFF_TIME_SEC_DEFAULT 120  // how much time the relay will remain off if the user asked

/* PIR Sensor Configuration */
#define PIR_IGNORE_AFTER_HOURS  22
#define PIR_IGNORE_AFTER_MIN    0
#define PIR_IGNORE_UNTIL_HOURS  6
#define PIR_IGNORE_UNTIL_MIN    0
#define PIR_IGNORE_AFTER        (PIR_IGNORE_AFTER_HOURS * 60 + PIR_IGNORE_AFTER_MIN) 
#define PIR_IGNORE_UNTIL        (PIR_IGNORE_UNTIL_HOURS * 60 + PIR_IGNORE_UNTIL_MIN)

/* global variables */
long relay_on_time = 0; // controls how much time the relay will remain on - this is the soft control
long relay_keep_off = 15; // controls how much time the relay will remain off - despite anything else

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
 * User Switch
 ****************************************/
#if USE_SWITCH == 1
void IRAM_ATTR switchChanged() 
{
  static unsigned long last_irq_time = 0;
  unsigned long irq_time = millis();

  if ((irq_time - last_irq_time) < SWITCH_DEBOUNCE_TIME_MS) {
    return;
  }
  last_irq_time = irq_time;

  switch_switched = true;
  
}

void setup_switch()
{
  /* set the switch pin to input */
  pinMode(SWITCH_PIN, INPUT_PULLDOWN);
  attachInterrupt(digitalPinToInterrupt(SWITCH_PIN), switchChanged, CHANGE);
  switch_switched = false;
}
#endif

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
        if (dht_data.humidity >= HUMIDITY_LIMIT_HIGH_DEFAULT) { // high enough to turn the relay on
          humidity_high = true;
          if (!led.isBlueOn()) {
            snprintf(msg, 50, "humidity high: %.1f %%", dht_data.humidity);
            debug(msg);
          }
          led.blueOn();
        } else if (dht_data.humidity <= HUMIDITY_LIMIT_LOW_DEFAULT) { // low enough to stop controlling
          humidity_high = false;        
          if (led.isBlueOn()) {
            snprintf(msg, 50, "humidity low: %.1f %%", dht_data.humidity);
            debug(msg);
          }
          led.blueOff();
        } else {
          //between HUMIDITY_LIMIT_HIGH_DEFAULT and HUMIDITY_LIMIT_LOW_DEFAULT
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

      // Calculate if current time is between ignore intervals
      int now = timeClient.getHours() * 60 + timeClient.getMinutes();

      if (PIR_IGNORE_AFTER < PIR_IGNORE_UNTIL) {
        // Interval does not cross midnight
        if (now >= PIR_IGNORE_AFTER && now < PIR_IGNORE_UNTIL) ignore = true;
      } else {
        // Interval crosses midnight
        if (now >= PIR_IGNORE_AFTER || now < PIR_IGNORE_UNTIL) ignore = true;
      }

      if (ignore) {
        debug("Movement ignored due to time interval");
      } else {
        debug("Movement detected");
        if(xSemaphoreTake(semph_relay, pdMS_TO_TICKS(100)) == pdTRUE ) {
          if (relay_keep_off == 0) { // the relay is not off
            relay_on_time = RELAY_ON_TIME_SEC_DEFAULT;
          }
          xSemaphoreGive(semph_relay);
        } else {
          debug("control_loop - movement: failed to take mutex");
        }
      }
    }

    // switch check
#if USE_SWITCH == 1    
    if (switch_switched) {
      switch_switched = false;
      
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
  led.redOn();   // Red LED on during startup
  led.greenOff();
  led.blueOff();

  setup_debug();
  
  /* WiFi Setup - FIRST after delay to ensure lwIP is ready */
  debug_nonFreeRTOS("Initializing WiFi...");
  wifiHandler.begin(&led);

  /* OTA Update stuff */
  wifiHandler.setupOTA();

  // semaphore for the relay_on_time
  semph_relay = xSemaphoreCreateMutex();
  if (semph_relay == NULL) {
    debug_nonFreeRTOS("Could not create the Relay Mutex!");
  }
  xSemaphoreGive(semph_relay);

  /* control loop */
  setup_control();

  /* setup the switch */
#if USE_SWITCH == 1
  setup_switch();
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
  wifiHandler.getTimeClient().update();

  wifiHandler.updateOTA();
}
