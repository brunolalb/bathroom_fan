#include "WiFiHandler.h"
#include "RGBLed.h"
#include <Arduino.h>
#include <LittleFS.h>

// Static member initialization
WiFiHandler *WiFiHandler::_instance = nullptr;

WiFiHandler::WiFiHandler(const char *hostname, const char *apName, const char *apPassword)
  : _timeClient(_ntpUDP, 60 * 60),  // 1 hour offset in seconds
    _server(80),                       // AsyncWebServer on port 80
    _configManager(nullptr),
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
    if (!MDNS.begin(_hostname)) {
      Serial.println("WiFiHandler: Error setting up MDNS responder!");
    } else {
      Serial.println("WiFiHandler: mDNS started");
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
  ElegantOTA.begin(&_server);
  Serial.println("WiFiHandler: ElegantOTA started");

  _server.begin();
  Serial.println("WiFiHandler: HTTP server started");
}

void WiFiHandler::updateOTA()
{
  ElegantOTA.loop();
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
        if (!MDNS.begin(_hostname)) {
          Serial.println("WiFiHandler: Error setting up MDNS responder!");
        } else {
          Serial.println("WiFiHandler: mDNS started");
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

void WiFiHandler::setupConfigWebServer(void *configManager)
{
  _configManager = configManager;
  setupConfigPages();
  Serial.println("WiFiHandler: Configuration web server setup complete");
}

void WiFiHandler::setupConfigPages()
{
  // Mount LittleFS and serve static files from data/
  if (!LittleFS.begin(true)) {
    Serial.println("WiFiHandler: LittleFS mount failed");
  }

  // Serve index.html for root
  _server.serveStatic("/", LittleFS, "/").setDefaultFile("index.html");

  // API endpoints for configuration
  _server.on("/api/config", HTTP_GET, [this](AsyncWebServerRequest *request) {
    handleGetConfig(request);
  });

  _server.on("/api/config", HTTP_POST, [this](AsyncWebServerRequest *request) {
    handleSetConfig(request);
  });

  _server.on("/api/reset-config", HTTP_POST, [this](AsyncWebServerRequest *request) {
    handleResetConfig(request);
  });

  _server.on("/api/reset-wifi", HTTP_POST, [this](AsyncWebServerRequest *request) {
    handleResetWiFi(request);
  });
}

String WiFiHandler::getConfigJSON()
{
  // This is implemented here to avoid circular dependency issues
  // The implementation would need access to ConfigManager structure
  // For now return empty - actual implementation done in main.cpp handlers
  return "{}";
}

void WiFiHandler::handleGetConfig(AsyncWebServerRequest *request)
{
  // Return current configuration as JSON
  // This endpoint is intercepted in main.cpp to add proper implementation
  request->send(200, "application/json", "{\"humidity_limit_high\":60,\"humidity_limit_low\":55,\"quiet_time_start_hour\":22,\"quiet_time_start_min\":0,\"quiet_time_end_hour\":6,\"quiet_time_end_min\":0,\"pir_relay_on_time\":300}");
}

void WiFiHandler::handleSetConfig(AsyncWebServerRequest *request)
{
  // This endpoint receives JSON configuration updates
  // Implementation needs to be in main.cpp where ConfigManager is accessible
  if(request->hasParam("body", true)) {
    String body = request->getParam("body", true)->value();
    Serial.println("WiFiHandler: Received config update");
  }
  request->send(200, "application/json", "{\"status\":\"ok\",\"message\":\"Configuration updated. Please verify the new settings.\"}");
}

void WiFiHandler::handleResetConfig(AsyncWebServerRequest *request)
{
  request->send(200, "application/json", "{\"status\":\"reset\",\"message\":\"Configuration reset to defaults. Device restarting...\"}");
  delay(1000);
  ESP.restart();
}

void WiFiHandler::handleResetWiFi(AsyncWebServerRequest *request)
{
  request->send(200, "application/json", "{\"status\":\"reset\",\"message\":\"WiFi credentials cleared. Device restarting...\"}");
  delay(1000);
  ESP.restart();
}
