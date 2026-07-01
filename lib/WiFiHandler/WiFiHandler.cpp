#include "WiFiHandler.h"
#include "RGBLed.h"
#include "ConfigManager.h"
#include "HardwareConfig.h"
#include <Arduino.h>
#include <LittleFS.h>

// Static member initialization
WiFiHandler *WiFiHandler::_instance = nullptr;

WiFiHandler::WiFiHandler(const char *hostname, const char *apName)
  : _server(80),                       // AsyncWebServer on port 80
    _configManager(nullptr),
    _deviceConfig(nullptr),
    _hostname(hostname),
    _apName(apName),
    _wifiTaskHandle(nullptr),
    _led(nullptr)
{
  _instance = this;
}

bool WiFiHandler::begin(RGBLed *led)
{
  _led = led;

  // Configure WiFi mode and hostname
  WiFi.mode(WIFI_STA);
  WiFi.setHostname(_hostname);

  // Configure WiFiManager - ensure AP is visible and accessible
  _wifiManager.setConfigPortalBlocking(false);
  _wifiManager.setConfigPortalTimeout(300);           // 5 minutes for portal timeout
  _wifiManager.setConnectTimeout(30);                 // 30 seconds to try connecting
  _wifiManager.setHostname(_hostname);                // Set hostname for AP and STA
  _wifiManager.setDebugOutput(false);

  Serial.println("WiFiHandler: Attempting WiFiManager connection...");

  // This is a blocking call - it will not return until connected or timeout
  bool connected = _wifiManager.autoConnect(_apName);

  if (!connected) {
    Serial.println("WiFiHandler: Connection failed");
    if (_led) {
      _led->redOn();  // Red LED indicates WiFi disconnected
    }
  }

  // Initialize system NTP time synchronization with configTime()
  // Using UTC as base timezone - specific timezone will be applied via system time
  configTime(0, 0, NTP_SERVER);
  Serial.println("WiFiHandler: NTP time synchronization started");

  // Start WiFi monitoring task
  xTaskCreate(wifiTaskStatic,
              "WiFiHandler",      // Task name
              (uint32_t)4096,     // Stack size
              nullptr,            // Parameter
              (UBaseType_t)1,     // Priority
              &_wifiTaskHandle);  // Task handle

  return connected;
}

void WiFiHandler::loop()
{
  // This function should be called in the main loop to handle WiFiManager and OTA updates
  _wifiManager.process();
  ElegantOTA.loop();
  // Note: NTP time is automatically synchronized by system configTime()
}

bool WiFiHandler::isConnected() const
{
  return WiFi.isConnected();
}

bool WiFiHandler::isTimeSet() const
{
  // Check if time has been synchronized by looking at the system time
  // If time() returns a value greater than a known year (2020), then NTP has synced
  time_t now = time(nullptr);
  struct tm *timeinfo = localtime(&now);
  return (timeinfo->tm_year + 1900) > 2020;  // If year is after 2020, NTP has synced
}

int WiFiHandler::getHours() const
{
  time_t now = time(nullptr);
  struct tm *timeinfo = localtime(&now);
  return timeinfo->tm_hour;
}

int WiFiHandler::getMinutes() const
{
  time_t now = time(nullptr);
  struct tm *timeinfo = localtime(&now);
  return timeinfo->tm_min;
}

void WiFiHandler::applyTimezone(const char *posixTz)
{
  if (!posixTz || strlen(posixTz) == 0) {
    Serial.println("WiFiHandler: Invalid timezone string");
    return;
  }
  
  Serial.printf("WiFiHandler: Applying timezone: %s\n", posixTz);
  
  // configTzTime() sets up the timezone with automatic DST handling
  // Parameters: const char *tzinfo, const char *server1, const char *server2
  // We use the POSIX timezone string and NTP servers
  configTzTime(posixTz, NTP_SERVER);
  
  Serial.println("WiFiHandler: Timezone applied");
}

void WiFiHandler::setupOTA()
{
  ElegantOTA.begin(&_server);
  Serial.println("WiFiHandler: ElegantOTA started");

  _server.begin();
  Serial.println("WiFiHandler: HTTP server started");
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
  bool was_connected = false;
  // This task monitors WiFi status after initial connection
  // and handles reconnection if needed

  Serial.println("WiFiHandler: Monitoring WiFi status");

  while (1) {
    if ((!WiFi.isConnected())&&(was_connected)) {
      was_connected = false;
      if (_led) {
        _led->redOn();  // Red LED indicates WiFi disconnected
      }
    } else if (!was_connected) {
      was_connected = true;
      
      char msg[50];
      snprintf(msg, 50, "WiFi reconnected: %s", WiFi.localIP().toString().c_str());
      Serial.println(msg);
      if (_led) {
        _led->redOff();
      }
      if (!MDNS.begin(_hostname)) {
        Serial.println("WiFiHandler: Error setting up MDNS responder!");
      } else {
        Serial.println("WiFiHandler: mDNS restarted");
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

void WiFiHandler::setupConfigWebServer(void *configManager, void *deviceConfig)
{
  _configManager = configManager;
  _deviceConfig = deviceConfig;
  setupConfigPages();
  Serial.println("WiFiHandler: Configuration web server setup complete");
}

void WiFiHandler::setupConfigPages()
{
  // Mount LittleFS and serve static files from data/
  if (!LittleFS.begin(true)) {
    Serial.println("WiFiHandler: LittleFS mount failed");
  }

  // Serve index.html for root (direct send avoids spurious .gz lookup log)
  _server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(LittleFS, "/index.html", "text/html");
  });

  // API: GET /api/config - return current settings
  _server.on("/api/config", HTTP_GET, [this](AsyncWebServerRequest *request) {
    handleGetConfig(request);
  });

  // API: POST /api/config - update settings (JSON body handler)
  _server.on("/api/config", HTTP_POST,
    [this](AsyncWebServerRequest *request) {
      handleSetConfigBody(request);
      _configPostBody = "";
    },
    nullptr,
    [this](AsyncWebServerRequest *request, uint8_t *data, size_t len, size_t index, size_t total) {
      if (index == 0) _configPostBody = "";
      _configPostBody += String((char *)data, len);
    }
  );

  // API: GET /api/time - return current time
  _server.on("/api/time", HTTP_GET, [this](AsyncWebServerRequest *request) {
    handleGetTime(request);
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
  DeviceConfig *config = (DeviceConfig *)_deviceConfig;
  if (!config) {
    request->send(500, "application/json", "{\"error\":\"config not initialized\"}");
    return;
  }
  char json[450];
  snprintf(json, sizeof(json),
    "{\"humidity_limit_high\":%.1f,\"humidity_limit_low\":%.1f,"
    "\"quiet_time_start_hour\":%u,\"quiet_time_start_min\":%u,"
    "\"quiet_time_end_hour\":%u,\"quiet_time_end_min\":%u,"
    "\"pir_relay_on_time\":%u,\"timezone_posix\":\"%s\"}",
    config->humidity_limit_high, config->humidity_limit_low,
    config->quiet_time_start_hour, config->quiet_time_start_min,
    config->quiet_time_end_hour, config->quiet_time_end_min,
    config->pir_relay_on_time, config->timezone_posix);
  request->send(200, "application/json", json);
}

void WiFiHandler::handleGetTime(AsyncWebServerRequest *request)
{
  char json[100];
  snprintf(json, sizeof(json),
    "{\"hours\":%d,\"minutes\":%d,\"is_set\":%s}",
    getHours(), getMinutes(), isTimeSet() ? "true" : "false");
  request->send(200, "application/json", json);
}

void WiFiHandler::handleSetConfigBody(AsyncWebServerRequest *request)
{
  ConfigManager *cm = (ConfigManager *)_configManager;
  DeviceConfig *config = (DeviceConfig *)_deviceConfig;

  if (!cm || !config) {
    request->send(500, "application/json", "{\"status\":\"error\",\"message\":\"Config not initialized\"}");
    return;
  }

  // Simple flat-JSON field extractor (no external dependency needed)
  auto getFloat = [&](const char *key) -> float {
    String search = String('"') + key + "\":";    
    int idx = _configPostBody.indexOf(search);
    if (idx < 0) return NAN;
    return _configPostBody.substring(idx + search.length()).toFloat();
  };
  auto getInt = [&](const char *key) -> int {
    String search = String('"') + key + "\":";    
    int idx = _configPostBody.indexOf(search);
    if (idx < 0) return -1;
    return _configPostBody.substring(idx + search.length()).toInt();
  };
  auto getString = [&](const char *key) -> String {
    String search = String('"') + key + "\":\"";    
    int idx = _configPostBody.indexOf(search);
    if (idx < 0) return "";
    int start = idx + search.length();
    int end = _configPostBody.indexOf('"', start);
    if (end < 0) end = start + 127;  // Fallback
    return _configPostBody.substring(start, end);
  };

  float hh  = getFloat("humidity_limit_high");
  float hl  = getFloat("humidity_limit_low");
  int   qsh = getInt("quiet_time_start_hour");
  int   qsm = getInt("quiet_time_start_min");
  int   qeh = getInt("quiet_time_end_hour");
  int   qem = getInt("quiet_time_end_min");
  int   pot = getInt("pir_relay_on_time");
  String tz = getString("timezone_posix");

  if (!isnan(hh))  config->humidity_limit_high    = hh;
  if (!isnan(hl))  config->humidity_limit_low     = hl;
  if (qsh >= 0)    config->quiet_time_start_hour  = (uint16_t)qsh;
  if (qsm >= 0)    config->quiet_time_start_min   = (uint16_t)qsm;
  if (qeh >= 0)    config->quiet_time_end_hour    = (uint16_t)qeh;
  if (qem >= 0)    config->quiet_time_end_min     = (uint16_t)qem;
  if (pot > 0)     config->pir_relay_on_time      = (uint16_t)pot;
  if (tz.length() > 0) {
    strncpy(config->timezone_posix, tz.c_str(), sizeof(config->timezone_posix) - 1);
    config->timezone_posix[sizeof(config->timezone_posix) - 1] = '\0';
  }

  if (cm->saveConfig(*config)) {
    // Apply timezone change
    if (tz.length() > 0) {
      applyTimezone(config->timezone_posix);
    }
    request->send(200, "application/json", "{\"status\":\"ok\",\"message\":\"Configuration saved.\"}" );
  } else {
    request->send(500, "application/json", "{\"status\":\"error\",\"message\":\"Failed to save configuration.\"}");
  }
}

void WiFiHandler::handleResetConfig(AsyncWebServerRequest *request)
{
  ConfigManager *cm = (ConfigManager *)_configManager;
  DeviceConfig *config = (DeviceConfig *)_deviceConfig;

  if (!cm || !config) {
    request->send(500, "application/json", "{\"status\":\"error\",\"message\":\"Config not initialized\"}");
    return;
  }

  if (cm->resetConfig(*config)) {
    // Apply default timezone
    applyTimezone(config->timezone_posix);
    request->send(200, "application/json", "{\"status\":\"ok\",\"message\":\"Configuration reset to defaults.\"}");
  } else {
    request->send(500, "application/json", "{\"status\":\"error\",\"message\":\"Failed to reset configuration.\"}");
  }
}

void WiFiHandler::handleResetWiFi(AsyncWebServerRequest *request)
{
  ConfigManager *cm = (ConfigManager *)_configManager;

  if (!cm) {
    request->send(500, "application/json", "{\"status\":\"error\",\"message\":\"ConfigManager not initialized\"}");
    return;
  }

  if (cm->resetWiFiCredentials()) {
    request->send(200, "application/json", "{\"status\":\"ok\",\"message\":\"WiFi credentials cleared. Device will restart...\"}");
    delay(1000);
    ESP.restart();
  } else {
    request->send(500, "application/json", "{\"status\":\"error\",\"message\":\"Failed to clear WiFi credentials.\"}");
  }
}
