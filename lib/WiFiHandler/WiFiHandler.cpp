#include "WiFiHandler.h"
#include "RGBLed.h"
#include <Arduino.h>

// Static member initialization
WiFiHandler *WiFiHandler::_instance = nullptr;

WiFiHandler::WiFiHandler(const char *hostname, const char *apName, const char *apPassword)
  : _timeClient(_ntpUDP, 60 * 60),  // 1 hour offset in seconds
    _server(80),                       // AsyncWebServer on port 80
    _otaTimer(nullptr),
    _hostname(hostname),
    _apName(apName),
    _apPassword(apPassword),
    _wifiTaskHandle(nullptr),
    _led(nullptr)
{
  _instance = this;
}

bool WiFiHandler::begin(RGBLed *led)
{
  _led = led;

  // Configure WiFi mode and hostname
  Serial.println("WiFiHandler: Configuring WiFi mode and hostname...");
  WiFi.mode(WIFI_STA);
  delay(100);
  WiFi.setHostname(_hostname);
  delay(100);

  // Configure WiFiManager
  _wifiManager.setConfigPortalTimeout(180);
  _wifiManager.setConnectTimeout(30);
  _wifiManager.setDebugOutput(false);

  Serial.println("WiFiHandler: Attempting WiFiManager connection...");

  // This is a blocking call - it will not return until connected or timeout
  bool connected = false;
  try {
    connected = _wifiManager.autoConnect(_apName, _apPassword);
  } catch (...) {
    Serial.println("WiFiHandler: Exception during WiFiManager");
    connected = false;
  }

  if (connected) {
    char msg[60];
    snprintf(msg, 60, "WiFi connected: %s", WiFi.localIP().toString().c_str());
    Serial.println(msg);
    if (_led) {
      _led->redOff();  // Turn off red LED when connected
    }
  } else {
    Serial.println("WiFiHandler: Connection failed, will retry in monitor task");
    if (_led) {
      _led->redOn();  // Red LED indicates WiFi disconnected
    }
  }

  // Initialize NTP time client
  _timeClient.begin();

  // Start WiFi monitoring task
  xTaskCreate(wifiTaskStatic,
              "WiFiHandler",      // Task name
              (uint32_t)4096,     // Stack size
              nullptr,            // Parameter
              (UBaseType_t)1,     // Priority
              &_wifiTaskHandle);  // Task handle

  return connected;
}

bool WiFiHandler::isConnected() const
{
  return WiFi.isConnected();
}

bool WiFiHandler::isTimeSet() const
{
  return _timeClient.isTimeSet();
}

int WiFiHandler::getHours() const
{
  return _timeClient.getHours();
}

int WiFiHandler::getMinutes() const
{
  return _timeClient.getMinutes();
}

NTPClient& WiFiHandler::getTimeClient()
{
  return _timeClient;
}

void WiFiHandler::setupOTA()
{
  // Create OTA timer (5 second delay before starting OTA service)
  _otaTimer = xTimerCreate("otaTimer", pdMS_TO_TICKS(5000), pdFALSE, (void*)0, otaTimerCallbackStatic);
  
  if (_otaTimer == nullptr) {
    Serial.println("WiFiHandler: ERROR - Failed to create OTA timer!");
    return;
  }
  
  // Setup root endpoint
  _server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(200, "text/plain", "Hi! This is ElegantOTA AsyncDemo.");
  });
  
  _server.begin();
  Serial.println("WiFiHandler: HTTP server started");
  
  // Start OTA timer if WiFi is connected
  if (isConnected() && _otaTimer != nullptr) {
    xTimerStart(_otaTimer, 0);
  }
}

void WiFiHandler::updateOTA()
{
  ElegantOTA.loop();
}

void WiFiHandler::setOtaTimerPaused(bool pause)
{
  if (_otaTimer == nullptr) return;

  if (pause) {
    xTimerStop(_otaTimer, 0);
  } else {
    xTimerStart(_otaTimer, 0);
  }
}

void WiFiHandler::otaTimerCallbackStatic(TimerHandle_t xTimer)
{
  if (_instance) {
    _instance->otaTimerCallback();
  }
}

void WiFiHandler::otaTimerCallback()
{
  reconnectToOta();
}

void WiFiHandler::reconnectToOta()
{
  // Setup mDNS for hostname resolution
  if (!MDNS.begin(_hostname)) {
    Serial.println("WiFiHandler: Error setting up MDNS responder!");
  } else {
    Serial.println("WiFiHandler: mDNS started");
  }

  // Start ElegantOTA
  ElegantOTA.begin(&_server);
  Serial.println("WiFiHandler: ElegantOTA started");
}

void WiFiHandler::wifiTaskStatic(void *param)
{
  if (_instance) {
    _instance->wifiTask();
  }
  vTaskDelete(nullptr);
}

void WiFiHandler::wifiTask()
{
  // This task monitors WiFi status after initial connection
  // and handles reconnection if needed

  Serial.println("WiFiHandler: Monitoring WiFi status");

  while (1) {
    if (!WiFi.isConnected()) {
      if (_led) {
        _led->redOn();  // Red LED indicates WiFi disconnected
      }
      Serial.println("WiFi disconnected, attempting reconnection...");

      // Pause OTA timer while disconnected
      if (_otaTimer != nullptr) {
        xTimerStop(_otaTimer, 0);
      }

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
        Serial.println(msg);
        if (_led) {
          _led->redOff();
        }
        
        // Resume OTA timer after reconnection
        if (_otaTimer != nullptr) {
          xTimerStart(_otaTimer, 0);
        }
      } else {
        Serial.println("Reconnection failed, will retry...");
      }
    }

    vTaskDelay(pdMS_TO_TICKS(5000)); // Check WiFi status every 5 seconds
  }
}

WiFiHandler::~WiFiHandler()
{
  if (_wifiTaskHandle != nullptr) {
    vTaskDelete(_wifiTaskHandle);
  }
}
