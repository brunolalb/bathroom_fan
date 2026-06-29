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
// wifi stuff
#include <WiFi.h> // official from esp32 lib (<2.0.0)
#include <WiFiManager.h> // tzapu WiFiManager
// FreeRTOS - official FreeRTOS lib
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/timers.h"
#include "freertos/event_groups.h"
// ota
#include <ESPmDNS.h> // comes with the ESP32 lib
#include <AsyncTCP.h> // by ESP32Async
#include <ESPAsyncWebServer.h> // by ESP32Async
#include <ElegantOTA.h> // by Ayush Sharma - set ELEGANTOTA_USE_ASYNC_WEBSERVER to 1 in the library
// DHT sensor
#include <DHT.h> // official from Adafruit
// NTP - time keeping
#include <NTPClient.h>
#include <WiFiUdp.h>

// debug stuff
SemaphoreHandle_t semph_debug; // controls access to the debug stuff
//#define EXAUSTOR_TEST

/****************************************
 * Forward Declarations
 ****************************************/
void debug(const char *msg);
void debug_nonFreeRTOS(const char *msg);
void reconnectToOta();
void movementDetected();
void DHTTask(void *param);
void control_loop(void *params);
void WiFiTask(void *param);

/* WiFi Manager */
#ifdef EXAUSTOR_TEST
#define WIFI_HOSTNAME "exaustor_test"
#define WIFI_AP_NAME "exaustor-test-setup"
#else
#define WIFI_HOSTNAME "exaustor"
#define WIFI_AP_NAME "exaustor-setup"
#endif
WiFiManager wifiManager;

/* OTA Update */
TimerHandle_t otaReconnectTimer;
AsyncWebServer server(80);  // Keeping AsyncWebServer for other endpoints

/****************************************
 * OTA Updates - Simplified (ElegantOTA disabled due to compatibility)
 ****************************************/
void reconnectToOta()
{
  /*use mdns for host name resolution*/
  if (!MDNS.begin(WIFI_HOSTNAME)) { //http://<hostname>.local
    debug("Error setting up MDNS responder!");
  }

  // ElegantOTA is disabled due to AsyncWebServer compatibility issues
  // You can still access /time endpoint for basic functionality
  debug("mDNS started.");

  // Start ElegantOTA
  ElegantOTA.begin(&server);
  debug("Elegant OTA started.");
  
  server.begin();
}

// Wrapper for OTA timer callback
void otaTimerCallback(TimerHandle_t xTimer)
{
  reconnectToOta();
}

void setup_OTA_Updates()
{
  otaReconnectTimer = xTimerCreate("otaTimer", pdMS_TO_TICKS(5000), pdFALSE, (void*)0, otaTimerCallback);
  
  if (otaReconnectTimer == NULL) {
    debug_nonFreeRTOS("ERROR: Failed to create OTA timer!");
    return;
  }
  
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(200, "text/plain", "Hi! This is ElegantOTA AsyncDemo.");
  });
  server.begin();
  debug_nonFreeRTOS("HTTP server started");
}

/* define DHT pins */
#define DHTPIN    19
#define DHTTYPE   DHT11
DHT dht(DHTPIN, DHTTYPE);
QueueHandle_t dht_queue = NULL;
typedef struct {
  float humidity;
  float temperature;  
} dht_queue_t;

/* RGB LED GPIO pins */
#define LED_R_PIN     2
#define LED_R_ON()    digitalWrite(LED_R_PIN, LOW)
#define LED_R_OFF()   digitalWrite(LED_R_PIN, HIGH)
#define LED_G_PIN     5
#define LED_G_ON()    digitalWrite(LED_G_PIN, LOW)
#define LED_G_OFF()   digitalWrite(LED_G_PIN, HIGH)
#define LED_B_PIN     21
#define LED_B_ON()    digitalWrite(LED_B_PIN, LOW)
#define LED_B_OFF()   digitalWrite(LED_B_PIN, HIGH)
#define LED_B_IS_ON() (!digitalRead(LED_B_PIN))

/* PIR Sensor */
#define PIR_PIN       18
bool movement_detected = false;
#define PIR_IGNORE_AFTER_HOURS  22
#define PIR_IGNORE_AFTER_MIN    0
#define PIR_IGNORE_UNTIL_HOURS  6
#define PIR_IGNORE_UNTIL_MIN    0
#define PIR_IGNORE_AFTER        (PIR_IGNORE_AFTER_HOURS * 60 + PIR_IGNORE_AFTER_MIN) 
#define PIR_IGNORE_UNTIL        (PIR_IGNORE_UNTIL_HOURS * 60 + PIR_IGNORE_UNTIL_MIN)

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

// NTPClient
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, 60*60); // offset in seconds

/* Application */
#define CONTROL_LOOP_PERIOD_MS       1000 // everything happens every 1 second
#define CONTROL_PERIOD_SEC_DEFAULT   10  // 10sec
#define CONTROL_PERIOD_SEC_MIN       1   // 1sec

#define DHT_PERIOD_SEC_DEFAULT      CONTROL_PERIOD_SEC_DEFAULT // period to read the DHT
#define HUMIDITY_LIMIT_HIGH_DEFAULT 60.0        // anything higher will trigger the relay
#define HUMIDITY_LIMIT_LOW_DEFAULT  55.0        // anything lower than this and the relay will shutoff

#define RELAY_ON_TIME_SEC_DEFAULT       300  // if the relay was turned on by presence (or switch), it'll stay on for this much time
#define RELAY_ON_TIME_SEC_HUMIDITY      60   // if humidity is high, relay will be on for this many seconds
#define RELAY_ON_TIME_SEC_MIN           30   // minimum time the relay will be on
#define RELAY_KEEP_OFF_TIME_SEC_DEFAULT 120  // how much time the relay will remain off if the user asked

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
 * WiFi Manager (Asynchronous)
 ****************************************/

void WiFiTask(void *param)
{
  // This task monitors WiFi status after initial connection
  // and handles reconnection if needed
  
  debug_nonFreeRTOS("WiFiTask: Monitoring WiFi status");
  
  while (1) {
    if (!WiFi.isConnected()) {
      LED_R_ON();  // Red LED indicates WiFi disconnected
      debug("WiFi disconnected, attempting reconnection...");
      
      // Pause OTA while disconnected
      xTimerStop(otaReconnectTimer, 0);
      
      // Try to reconnect
      WiFi.reconnect();
      
      // Wait for reconnection with timeout
      int timeout = 20;
      while (!WiFi.isConnected() && timeout > 0) {
        vTaskDelay(pdMS_TO_TICKS(1000));
        timeout--;
      }
      
      if (WiFi.isConnected()) {
        char msg[50];
        snprintf(msg, 50, "WiFi reconnected: %s", WiFi.localIP().toString().c_str());
        debug(msg);
        LED_R_OFF();
        
        // Resume OTA timer
        if (otaReconnectTimer != NULL) {
          xTimerStart(otaReconnectTimer, 0);
        }
      } else {
        debug("Reconnection failed, will retry...");
      }
    }
    
    vTaskDelay(pdMS_TO_TICKS(5000)); // Check WiFi status every 5 seconds
  }
  
  vTaskDelete(NULL); // Should never reach here
}

void setup_WiFi() 
{
  debug_nonFreeRTOS("setup_WiFi: Configuring WiFi mode and hostname...");
  
  // Very basic WiFi setup - avoid complex WiFiManager calls
  WiFi.mode(WIFI_STA);
  delay(100);
  
  WiFi.setHostname(WIFI_HOSTNAME);
  delay(100);
  
  // Try to connect using WiFiManager with minimal options
  wifiManager.setConfigPortalTimeout(180);
  wifiManager.setConnectTimeout(30);
  wifiManager.setDebugOutput(false);
  
  debug_nonFreeRTOS("setup_WiFi: Attempting WiFiManager connection...");
  
  // This is a blocking call - it will not return until connected or timeout
  bool connected = false;
  try {
    connected = wifiManager.autoConnect(WIFI_AP_NAME, "admin");
  } catch (...) {
    debug_nonFreeRTOS("setup_WiFi: Exception during WiFiManager");
    connected = false;
  }
  
  if (connected) {
    char msg[60];
    snprintf(msg, 60, "WiFi connected: %s", WiFi.localIP().toString().c_str());
    debug_nonFreeRTOS(msg);
    LED_R_OFF();  // Turn off red LED when connected
    
    // Start OTA timer once WiFi is connected
    if (otaReconnectTimer != NULL) {
      xTimerStart(otaReconnectTimer, 0);
    } else {
      debug_nonFreeRTOS("WARNING: OTA timer not initialized");
    }
  } else {
    debug_nonFreeRTOS("setup_WiFi: Connection failed, will retry in monitor task");
    LED_R_ON();  // Red LED indicates WiFi disconnected
  }
  
  debug_nonFreeRTOS("setup_WiFi: Initial connection attempt complete");
  
  // Create WiFi monitoring task after WiFi setup is complete
  xTaskCreate(WiFiTask,
            "WiFi_Monitor",
            8000,
            NULL,
            1,
            NULL);
}

/****************************************
 * Movement Sensor
 ****************************************/

void IRAM_ATTR movementDetected() 
{
  movement_detected = true;
}

void setup_movement_sensor()
{
  /* setup the pir sensor */
  pinMode(PIR_PIN, INPUT_PULLUP);
  movement_detected = false;
  attachInterrupt(digitalPinToInterrupt(PIR_PIN), movementDetected, RISING);
}

/****************************************
 * Humidity/Temperature Sensor (DHT11)
 ****************************************/

void DHTTask(void *param)
{
  TickType_t xLastWakeTime = xTaskGetTickCount();
  float humidity_local = 50.0;    // initialized for EXAUSTOR_TEST mode
  float temperature_local = 25.0; // initialized for EXAUSTOR_TEST mode
  dht_queue_t queue_data;
  char msg[30];
  
  while(1) {
    queue_data.humidity = -1.0;
    queue_data.temperature = -273.0;
    
    /* read DHT11 sensor */
#ifdef EXAUSTOR_TEST
    humidity_local = humidity_local + (float(random(-50, 50)) / 10.0);
    if (humidity_local < 50.0) humidity_local = 50.0;
    if (humidity_local > 65.0) humidity_local = 56.0;
    temperature_local = temperature_local + (float(random(-10, 10)) / 10.0);
    if (temperature_local < 10.0) temperature_local = 25.0;
#else    
    humidity_local = dht.readHumidity();
    temperature_local = dht.readTemperature();
#endif

    if (isnan(humidity_local)) {
      debug("Failed to read humidity");
    } else if (isnan(temperature_local)) {
      debug("Failed to read temperature");
    } else {
      snprintf(msg, 30, "DHT data: %.1f %%, %.1f C", humidity_local, temperature_local);
      debug(msg);

      queue_data.humidity = humidity_local;
      queue_data.temperature = temperature_local;
      if (dht_queue) {
        if (xQueueSend(dht_queue, (void*)&queue_data, pdMS_TO_TICKS(100)) != pdPASS) {
          debug("failed to send data to dht queue");
        }
      }
    }

    vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(DHT_PERIOD_SEC_DEFAULT * 1000));    
  } // while(1)

}

void setup_DHT()
{
  dht.begin();

  dht_queue = xQueueCreate(5, sizeof(dht_queue_t));
  if (dht_queue == NULL) {
    debug_nonFreeRTOS("failed to create dht queue");    
  }

  xTaskCreate(DHTTask,
            "DHT",   // A name just for humans
            10000,  // Stack size
            NULL,
            2,  // priority
            NULL );
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
  dht_queue_t dht_data;

  while (1) {
    // humidity check
    dht_data.humidity = -1.0;
    dht_data.temperature = -273.0;
    if ((dht_queue) && (xQueueReceive(dht_queue, &dht_data, pdMS_TO_TICKS(100)) == pdPASS)) {
      if (!isnan(dht_data.humidity) && (dht_data.humidity >= 0)) {
        // check if humidity within ranges
        if (dht_data.humidity >= HUMIDITY_LIMIT_HIGH_DEFAULT) { // high enough to turn the relay on
          humidity_high = true;
          if (!LED_B_IS_ON()) {
            snprintf(msg, 50, "humidity high: %.1f %%", dht_data.humidity);
            debug(msg);
          }
          LED_B_ON();
        } else if (dht_data.humidity <= HUMIDITY_LIMIT_LOW_DEFAULT) { // low enough to stop controlling
          humidity_high = false;        
          if (LED_B_IS_ON()) {
            snprintf(msg, 50, "humidity low: %.1f %%", dht_data.humidity);
            debug(msg);
          }
          LED_B_OFF();
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
    if (movement_detected && timeClient.isTimeSet()) {
      movement_detected = false;

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
        LED_G_OFF();        
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
        LED_G_ON();
      } else {
        if (RELAY_IS_ON()) {
          relay_keep_off = 5;
          debug("IDLE");
        }          
        RELAY_OFF();
        LED_G_OFF();
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
  pinMode(LED_R_PIN, OUTPUT); LED_R_ON();   // Red LED on during startup
  pinMode(LED_G_PIN, OUTPUT); LED_G_OFF();
  pinMode(LED_B_PIN, OUTPUT); LED_B_OFF();

  setup_debug();
  
  /* WiFi Setup - FIRST after delay to ensure lwIP is ready */
  debug_nonFreeRTOS("Initializing WiFi...");
  setup_WiFi();

  timeClient.begin();

  /* OTA Update stuff */
  setup_OTA_Updates();

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
  setup_DHT();  

  /* setup movement sensor */
  setup_movement_sensor();
  
  /* WiFi Setup - LAST, after all other systems initialized */
  debug_nonFreeRTOS("All systems initialized, starting WiFi task...");
  setup_WiFi();

  debug_nonFreeRTOS("Setup complete - all systems initialized");
}

/****************************************
 * Application Loop
 ****************************************/
void loop() 
{
  timeClient.update();

  ElegantOTA.loop();
}
