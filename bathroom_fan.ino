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
I use MQTT to report and control this module, so remember to update
  the WiFi network name and password, as well as your MQTT Server address
My MQTT Server runs Mosquitto
I also have a NodeRED MQTT Dashboard for fancy reporting
*/


#include <stdio.h>
// wifi stuff
#include <WiFi.h> // official from esp32 lib (<2.0.0)
// FreeRTOS - official FreeRTOS lib
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/timers.h"
#include "freertos/event_groups.h"
// mqtt
#include <AsyncMqttClient.h> // https://github.com/marvinroger/async-mqtt-client
// ota
#include <ESPmDNS.h> // comes with the ESP32 lib
#include <AsyncElegantOTA.h> // official lib: AsyncElegantOTA
#include <AsyncTCP.h> // https://github.com/me-no-dev/AsyncTCP
#include <ESPAsyncWebServer.h> // https://github.com/me-no-dev/ESPAsyncWebServer
// DHT sensor
#include <DHT.h> // official from Adafruit

// debug stuff
SemaphoreHandle_t semph_debug; // controls access to the debug stuff
//#define EXAUSTOR_TEST

/* WiFi */
#ifdef EXAUSTOR_TEST
#define WIFI_HOSTNAME "exaustor_test"
#else
#define WIFI_HOSTNAME "exaustor"
#endif
const char* WIFI_SSID = "I Believe Wi Can Fi";
const char* WIFI_PASSWORD = "";
TimerHandle_t wifiReconnectTimer;

/* OTA Update */
TimerHandle_t otaReconnectTimer;
AsyncWebServer server(80);

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

/* MQTT */
AsyncMqttClient mqttClient;
TimerHandle_t mqttReconnectTimer;
SemaphoreHandle_t semph_mqtt;
// some config
#define MQTT_MY_NAME      WIFI_HOSTNAME
#define MQTT_SERVER_IP    IPAddress(192, 168, 1, 42)
#define MQTT_SERVER_PORT  1883
#define MQTT_PUBLISH_RETAINED   true  // if true, will publish all messages with the retain flag
#define MQTT_PUBLISH_QOS        0     // QOS level for published messages
#define MQTT_SUBSCRIBE_QOS      0     // QOS level for subscribed topics
// topics
#define MQTT_TOPIC_HOME         "smarthome/bathroom/"
#define MQTT_TOPIC_BASE_ADDRESS MQTT_TOPIC_HOME MQTT_MY_NAME
#define MQTT_TOPIC_DEBUG        MQTT_TOPIC_BASE_ADDRESS "/debug"        // text
#define MQTT_TOPIC_LED_R        MQTT_TOPIC_BASE_ADDRESS "/led/red"      // 1 or 0
#define MQTT_TOPIC_LED_G        MQTT_TOPIC_BASE_ADDRESS "/led/green"    // 1 or 0
#define MQTT_TOPIC_LED_B        MQTT_TOPIC_BASE_ADDRESS "/led/blue"     // 1 or 0
#define MQTT_TOPIC_HUMIDITY     MQTT_TOPIC_BASE_ADDRESS "/humidity"     // float
#define MQTT_TOPIC_TEMPERATURE  MQTT_TOPIC_BASE_ADDRESS "/temperature"  // float
#define MQTT_TOPIC_RELAY_WRITE  MQTT_TOPIC_BASE_ADDRESS "/on_off_feedback" // 1 or 0 - to MQTT
#define MQTT_TOPIC_RELAY_READ   MQTT_TOPIC_BASE_ADDRESS "/on_off"       // 1 or 0 - from MQTT
#define MQTT_TOPIC_ON_REASON    MQTT_TOPIC_BASE_ADDRESS "/reason_on"    // string
// buffer
char global_buffer[20];

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

typedef enum _on_reason_e {
  on_movement,
  on_switch,
  on_humidity,
  on_user,

  on_unknown = 255
} on_reason_e;

/* global variables */
long relay_on_time = 0; // controls how much time the relay will remain on - this is the soft control
long relay_keep_off = 15; // controls how much time the relay will remain off - despite anything else
on_reason_e on_reason = on_unknown;

SemaphoreHandle_t semph_relay; // controls access to the relay_on_time and relay_keep_off

/****************************************
 * Debug
 ****************************************/

void debug(char *msg) 
{
  if (xPortInIsrContext()) {
    Serial.print("ISR!!");
    Serial.println(msg);
    return;
  }

  if(xSemaphoreTake(semph_debug, pdMS_TO_TICKS(100)) == pdTRUE ) {
    mqtt_publish(MQTT_TOPIC_DEBUG, msg);
    Serial.println(msg);
    xSemaphoreGive(semph_debug);    
  }  
}

void debug_nonFreeRTOS(char *msg)
{
  Serial.println(msg);
}

void setup_debug()
{
  semph_debug = xSemaphoreCreateMutex();
  xSemaphoreGive(semph_debug);
}

/****************************************
 * WiFi
 ****************************************/

void setup_WiFi() 
{
  char msg[50];
  snprintf(msg, 50, "Connecting to %s", WIFI_SSID);
  debug_nonFreeRTOS(msg);
  
  // delete old config
  WiFi.disconnect(true);

  delay(1000);

  WiFi.onEvent(WiFiStationStarted, SYSTEM_EVENT_STA_START);
  WiFi.onEvent(WiFiStationConnected, SYSTEM_EVENT_STA_CONNECTED);
  WiFi.onEvent(WiFiGotIP, SYSTEM_EVENT_STA_GOT_IP);
  WiFi.onEvent(WiFiStationDisconnected, SYSTEM_EVENT_STA_DISCONNECTED);

  WiFi.mode(WIFI_STA);

  // one shot timer
  wifiReconnectTimer = xTimerCreate("wifiTimer", pdMS_TO_TICKS(2000), pdFALSE, (void*)0, reinterpret_cast<TimerCallbackFunction_t>(reconnectToWifi));
}

void reconnectToWifi()
{
  debug("Reconnecting to Wifi...");
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);  
}

void WiFiStationStarted(WiFiEvent_t event, WiFiEventInfo_t info)
{
  debug("Station Started");
  WiFi.setHostname(WIFI_HOSTNAME);
}

void WiFiStationConnected(WiFiEvent_t event, WiFiEventInfo_t info)
{
  debug("Connected to AP successfully!");
}

void WiFiGotIP(WiFiEvent_t event, WiFiEventInfo_t info)
{
  char msg[30];

  LED_R_OFF();

  debug("WiFi connected");
  snprintf(msg, 30, "IP address: %s", WiFi.localIP().toString().c_str());
  debug(msg);
  snprintf(msg, 30, "RRSI: %i dB", WiFi.RSSI());
  debug(msg);

  // connect the mqtt again
  xTimerStart(mqttReconnectTimer, 0);
  // start the OTA again
  xTimerStart(otaReconnectTimer, 0);
}

void WiFiStationDisconnected(WiFiEvent_t event, WiFiEventInfo_t info)
{
  char msg[50];

  LED_R_ON();

  snprintf(msg, 50, "Disconnected from WiFi: %u", info.disconnected.reason);
  debug(msg);

  // stop the mqtt reconnect timer to ensure we don't reconnect to MQTT while reconnecting to Wi-Fi
  xTimerStop(mqttReconnectTimer, 0); 
  // stop the ota reconnect timer to ensure we don't reconnect to MQTT while reconnecting to Wi-Fi
  xTimerStop(otaReconnectTimer, 0); 
  // start the wifi reconnect timer
  xTimerStart(wifiReconnectTimer, 0);
}

/****************************************
 * MQTT
 ****************************************/
void setup_MQTT() 
{
  // one shot timer
  mqttReconnectTimer = xTimerCreate("mqttTimer", pdMS_TO_TICKS(2000), pdFALSE, (void*)0, reinterpret_cast<TimerCallbackFunction_t>(reconnectToMqtt));

  // semaphore to prevent simultaneous access
  semph_mqtt = xSemaphoreCreateMutex();
  xSemaphoreGive(semph_mqtt);  
  
  mqttClient.onConnect(onMqttConnect);
  mqttClient.onDisconnect(onMqttDisconnect);
  //mqttClient.onSubscribe(onMqttSubscribe);
  //mqttClient.onUnsubscribe(onMqttUnsubscribe);
  //mqttClient.onPublish(onMqttPublish);
  mqttClient.onMessage(onMqttMessage); // when we receive a message from a subscribed topic

  mqttClient.setServer(MQTT_SERVER_IP, MQTT_SERVER_PORT);
  mqttClient.setClientId(MQTT_MY_NAME);

  // wait for wifi to start, it will start the timer to connect to the mqtt server
}

void reconnectToMqtt() 
{
  debug("Connecting to MQTT...");
  mqttClient.connect();
}

void onMqttConnect(bool sessionPresent) 
{
  uint16_t packetIdSub;
  char msg[100];

  debug("Connected to MQTT.");
  snprintf(msg, 50, "Session present: %s", sessionPresent ? "True" : "False");
  
  /* subscribe to topics with default QoS 0*/
  packetIdSub = mqttClient.subscribe(MQTT_TOPIC_LED_R, MQTT_SUBSCRIBE_QOS);
  snprintf(msg, 100, "Subscribing at QoS %u, topic %s, packetId %u", MQTT_SUBSCRIBE_QOS, MQTT_TOPIC_LED_R, packetIdSub);
  debug(msg);
  packetIdSub = mqttClient.subscribe(MQTT_TOPIC_LED_G, MQTT_SUBSCRIBE_QOS);
  snprintf(msg, 100, "Subscribing at QoS %u, topic %s, packetId %u", MQTT_SUBSCRIBE_QOS, MQTT_TOPIC_LED_G, packetIdSub);
  debug(msg);
  packetIdSub = mqttClient.subscribe(MQTT_TOPIC_LED_B, MQTT_SUBSCRIBE_QOS);
  snprintf(msg, 100, "Subscribing at QoS %u, topic %s, packetId %u", MQTT_SUBSCRIBE_QOS, MQTT_TOPIC_LED_B, packetIdSub);
  debug(msg);
  packetIdSub = mqttClient.subscribe(MQTT_TOPIC_RELAY_READ, MQTT_SUBSCRIBE_QOS);
  snprintf(msg, 100, "Subscribing at QoS %u, topic %s, packetId %u", MQTT_SUBSCRIBE_QOS, MQTT_TOPIC_RELAY_READ, packetIdSub);
  debug(msg);
}

void onMqttDisconnect(AsyncMqttClientDisconnectReason reason) 
{
  debug("Disconnected from MQTT.");
  if (WiFi.isConnected()) {
    // didn't disconnect because the wifi stopped, so try to connect again    
    xTimerStart(mqttReconnectTimer, 0);
  }
}

bool mqtt_publish(char *topic, char *payload)
{
  if (xSemaphoreTake(semph_mqtt, pdMS_TO_TICKS(100)) == pdTRUE) {
    if (mqttClient.connected()) {
      mqttClient.publish(topic, MQTT_PUBLISH_QOS, MQTT_PUBLISH_RETAINED, payload);
      xSemaphoreGive(semph_mqtt);      
    } else {
      xSemaphoreGive(semph_mqtt);
      return false;
    }
  } else {
    return false;
  }
  return true;
}

void onMqttMessage(char* topic, char* payload, AsyncMqttClientMessageProperties properties, size_t len, size_t index, size_t total) 
{
  char msg[200];
  char new_payload[20];
  snprintf(new_payload, 20, "%s", payload);
  new_payload[len] = 0;
  snprintf(msg, 200, "Message received: %s\r\n payload (%u bytes): %s", 
                     topic, len, new_payload);
  debug(msg);
  
  if (String(topic) == String(MQTT_TOPIC_LED_R)) {
    (payload[0] == '1') ? LED_R_ON() : LED_R_OFF();
  } else if (String(topic) == String(MQTT_TOPIC_LED_G)) {
    (payload[0] == '1') ? LED_G_ON() : LED_G_OFF();
  } else if (String(topic) == String(MQTT_TOPIC_LED_B)) {
    (payload[0] == '1') ? LED_B_ON() : LED_B_OFF();
  } else if (String(topic) == String(MQTT_TOPIC_RELAY_READ)) {
    BaseType_t pxHigherPriorityTaskWoken = pdFALSE;    
    if(xSemaphoreTake(semph_relay, pdMS_TO_TICKS(100)) == pdTRUE ) {    
      if (payload[0] == '1') {
        relay_on_time = RELAY_ON_TIME_SEC_DEFAULT;
        relay_keep_off = 0;
        on_reason = on_user;
      } else {
        relay_on_time = 0;
        relay_keep_off = RELAY_KEEP_OFF_TIME_SEC_DEFAULT;
      }
      xSemaphoreGive(semph_relay);
    } else {
      debug("onMqttMessage: failed to get mutex");
    }
  } else {
    // i don't know ...
  }
}

void send_on_reason()
{
  char msg[20];
  char debug_msg[50];
  
  if (!mqttClient.connected()) {
    debug("send_on_reason: mqtt disconnected");
    return;
  }

  switch(on_reason) {
    case on_movement:
      snprintf(msg, 20, "Movement");
      break;
    case on_switch:
      snprintf(msg, 20, "Switch");
      break;
    case on_humidity:
      snprintf(msg, 20, "Humidity");
      break;
    case on_user:
      snprintf(msg, 20, "User");
      break;
    default:
      snprintf(msg, 20, "Unknown");
      break;
  }
  mqtt_publish(MQTT_TOPIC_ON_REASON, msg);

  snprintf(debug_msg, 50, "On Reason: %s", msg);
  debug(debug_msg);

  on_reason = on_unknown;
}

/****************************************
 * OTA Updates
 ****************************************/
void reconnectToOta()
{
  /*use mdns for host name resolution*/
  if (!MDNS.begin(WIFI_HOSTNAME)) { //http://<hostname>.local
    debug("Error setting up MDNS responder!");
    //return;
    /* while (1) {
      delay(1000);
    } */
  }

  AsyncElegantOTA.begin(&server);    // Start ElegantOTA
  debug("Elegant OTA started.");
  
  server.begin();
  debug("Webserver started.");
}

void setup_OTA_Updates()
{

  otaReconnectTimer = xTimerCreate("otaTimer", pdMS_TO_TICKS(5000), pdFALSE, (void*)0, reinterpret_cast<TimerCallbackFunction_t>(reconnectToOta));

  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(200, "text/plain", "Hi! I am ESP32.");
  });

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
  float humidity_local, temperature_local;
  dht_queue_t queue_data;
  char msg[30];
  
  while(1) {
    queue_data.humidity = -1.0;
    queue_data.temperature = -273.0;
    
    /* read DHT11 sensor and convert to string */
#ifdef EXAUSTOR_TEST
    humidity_local = humidity_local + (float(random(-50, 50)) / 10.0);
    if (humidity_local < 50.0) humidity_local = 50.0;
    if (humidity_local > 65.0) humidity_local = 56.0;
#else    
    humidity_local = dht.readHumidity();
#endif
    if (isnan(humidity_local)) {
      debug("Failed to read humidity");
    }
#ifdef EXAUSTOR_TEST
    temperature_local = temperature_local + (float(random(-10, 10)) / 10.0);
    if (temperature_local < 10.0) temperature_local = 25.0;
#else
    temperature_local = dht.readTemperature();
#endif
    if (isnan(temperature_local)) {
      debug("Failed to read temperature");
    } else {}

    snprintf(msg, 30, "DHT data: %.1f %%, %.1f C", humidity_local, temperature_local);
    debug(msg);

    if ((!isnan(humidity_local)) && (!isnan(temperature_local))) {    
      queue_data.humidity = humidity_local;
      queue_data.temperature = temperature_local;
      if (dht_queue) {
        if (xQueueSend(dht_queue, (void*)&queue_data, pdMS_TO_TICKS(100)) != pdPASS) {
          debug("failed to send data to dht queue");
        } else {}
      } else {}
    }

    vTaskDelayUntil(&xLastWakeTime, DHT_PERIOD_SEC_DEFAULT * 1000 / portTICK_PERIOD_MS);    
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

void send_relay_status()
{
  char msg[5];

  if (mqttClient.connected()) {
    msg[0] = RELAY_IS_ON() ? '1' : '0'; 
    msg[1] = 0;
    mqtt_publish(MQTT_TOPIC_RELAY_WRITE, msg);

    if (RELAY_IS_ON()) send_on_reason();
  }  

}

void control_loop(void *params)
{
  TickType_t xLastWakeTime = xTaskGetTickCount();
  uint16_t packetId;
  bool send_relay = false;
  char msg[50];
  bool humidity_high = false;  
  dht_queue_t dht_data;

  while (1) {
    // humidity check
    dht_data.humidity = -1.0;
    dht_data.temperature = -273.0;
    if ((dht_queue) && (xQueueReceive(dht_queue, &dht_data, pdMS_TO_TICKS(100)) == pdPASS)) {
      if (!isnan(dht_data.humidity) && (dht_data.humidity >= 0)) {
        // publish the humidity
        snprintf (msg, 20, "%.1lf", dht_data.humidity);
        mqtt_publish(MQTT_TOPIC_HUMIDITY, msg);        
        snprintf (msg, 20, "%.1lf", dht_data.temperature);
        mqtt_publish(MQTT_TOPIC_TEMPERATURE, msg);        
        // snprintf(msg, 50, "Published on topic %s, packetId %u: %s", MQTT_TOPIC_TEMPERATURE, packetId, msg);
        // debug(msg);
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
              on_reason = on_humidity;
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
    if (movement_detected) {
      movement_detected = false;
            
      debug("Movement detected");
      if(xSemaphoreTake(semph_relay, pdMS_TO_TICKS(100)) == pdTRUE ) {
        if (relay_keep_off == 0) { // the relay is not off
          relay_on_time = RELAY_ON_TIME_SEC_DEFAULT;
          on_reason = on_movement;
        }
        xSemaphoreGive(semph_relay);
      } else {
        debug("control_loop - movement: failed to take mutex");
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
          on_reason = on_switch;
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
        if (RELAY_IS_ON()) send_relay = true;
        RELAY_OFF();
        LED_G_OFF();        
      } else if (relay_on_time > 0) {
        if (relay_on_time % 10 == 0) {
          snprintf(msg, 50, "Relay on: %lu", relay_on_time);
          debug(msg);
        }
        relay_on_time--;
        if (!RELAY_IS_ON()) {
          send_relay = true;
          debug("Relay ON");
        }
        RELAY_ON();
        LED_G_ON();
      } else {
        if (RELAY_IS_ON()) {
          relay_keep_off = 5;
          send_relay = true;
          debug("IDLE");
        }          
        RELAY_OFF();
        LED_G_OFF();
      }
      xSemaphoreGive(semph_relay);
    } else {
      debug("control_loop: failed to take mutex");
    }

    if (send_relay) {
      send_relay = false;
      send_relay_status();      
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

  setup_debug();
  
  /* Wifi Setup */
  setup_WiFi();

  /* MQTT Setup */
  setup_MQTT();

  /* OTA Update stuff */
  setup_OTA_Updates();

  // start the wifi reconnect timer
  xTimerStart(wifiReconnectTimer, 0);

  // while we're connecting:
  /* set led as output to control led on-off */
  pinMode(LED_R_PIN, OUTPUT); LED_R_ON();
  pinMode(LED_G_PIN, OUTPUT); LED_G_OFF();
  pinMode(LED_B_PIN, OUTPUT); LED_B_OFF();

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

}

/****************************************
 * Application Loop
 ****************************************/
void loop() 
{

}
