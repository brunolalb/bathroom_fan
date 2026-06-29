#ifndef WIFIHANDLER_H
#define WIFIHANDLER_H

#include <WiFi.h>
#include <WiFiManager.h>
#include <WiFiUdp.h>
#include <NTPClient.h>
#include <ESPmDNS.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <ElegantOTA.h>
#include "freertos/FreeRTOS.h"
#include "freertos/timers.h"

// Forward declaration for LED control
class RGBLed;

class WiFiHandler {
public:
  /**
   * Constructor - Initializes WiFiHandler
   * @param hostname The hostname for WiFi connection
   * @param apName The AP name for WiFiManager captive portal
   * @param apPassword The password for WiFiManager captive portal
   */
  WiFiHandler(const char *hostname, const char *apName, const char *apPassword);

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
   * Update OTA (call ElegantOTA.loop() in main loop)
   */
  void updateOTA();

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
   * Get NTPClient instance for direct access to time functions
   * @return reference to NTPClient
   */
  NTPClient& getTimeClient();

  /**
   * Pause/Resume OTA timer
   * @param pause true to pause, false to resume
   */
  void setOtaTimerPaused(bool pause);

  /**
   * Destructor
   */
  ~WiFiHandler();

private:
  WiFiManager _wifiManager;
  WiFiUDP _ntpUDP;
  NTPClient _timeClient;
  AsyncWebServer _server;
  TimerHandle_t _otaTimer;
  
  const char *_hostname;
  const char *_apName;
  const char *_apPassword;
  TaskHandle_t _wifiTaskHandle;
  RGBLed *_led;
  
  // Static data for WiFi task and OTA timer callback
  static WiFiHandler *_instance;
  static void wifiTaskStatic(void *param);
  void wifiTask();
  
  static void otaTimerCallbackStatic(TimerHandle_t xTimer);
  void otaTimerCallback();
  void reconnectToOta();
};

#endif

