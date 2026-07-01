#ifndef WIFIHANDLER_H
#define WIFIHANDLER_H

#include <WiFi.h>
#include <WiFiManager.h>
#include <ESPmDNS.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <ElegantOTA.h>
#include <LittleFS.h>
#include <ctime>
#include "freertos/FreeRTOS.h"

// Forward declaration for LED control
class RGBLed;

class WiFiHandler {
public:
  /**
   * Constructor - Initializes WiFiHandler
   * @param hostname The hostname for WiFi connection
   * @param apName The AP name for WiFiManager captive portal
   */
  WiFiHandler(const char *hostname, const char *apName);

  /**
   * Initialize WiFi connection and NTP client
   * This is a blocking call that waits for WiFi connection
   * @param led Pointer to RGBLed instance for status indication
   * @return true if WiFi connected, false otherwise
   */
  bool begin(RGBLed *led);

  /**
   * Setup OTA updates with ElegantOTA
   * Must be called after begin()
   */
  void setupOTA();

  /**
   * Check if WiFi is currently connected
   * @return true if connected, false otherwise
   */
  bool isConnected() const;

  /**
   * Check if NTP time has been set
   * @return true if time is set, false otherwise
   */
  bool isTimeSet() const;

  /**
   * Get current hour (0-23)
   * @return current hour
   */
  int getHours() const;

  /**
   * Get current minute (0-59)
   * @return current minute
   */
  int getMinutes() const;

  /**
   * Setup configuration web interface with REST API
   * Must be called after begin()
   * @param configManager Pointer to ConfigManager instance
   * @param deviceConfig  Pointer to DeviceConfig instance
   */
  void setupConfigWebServer(void *configManager, void *deviceConfig);

  void loop();  // Call this in the main loop to handle WiFiManager and OTA updates

  /**
   * Destructor
   */
  ~WiFiHandler();

private:
  WiFiManager _wifiManager;
  AsyncWebServer _server;
  void *_configManager;  // Void pointer to avoid circular dependency
  void *_deviceConfig;   // Void pointer to avoid circular dependency
  String _configPostBody;
  
  const char *_hostname;
  const char *_apName;
  TaskHandle_t _wifiTaskHandle;
  RGBLed *_led;
  
  // Static data for WiFi task
  static WiFiHandler *_instance;
  static void wifiTaskStatic(void *param);
  void wifiTask();
  
  // Configuration web server handlers
  void setupConfigPages();
  String getConfigJSON();
  void handleGetConfig(AsyncWebServerRequest *request);
  void handleSetConfigBody(AsyncWebServerRequest *request);
  void handleResetConfig(AsyncWebServerRequest *request);
  void handleResetWiFi(AsyncWebServerRequest *request);
};

#endif

