#include "ConfigManager.h"
#include <Arduino.h>
#include <WiFiManager.h>

ConfigManager::ConfigManager()
  : _nvsHandle(0),
    _namespace("device_config")
{
}

bool ConfigManager::begin()
{
  // Initialize NVS
  esp_err_t err = nvs_flash_init();
  if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    // NVS partition was truncated and needs to be erased
    // Retry nvs_flash_init
    nvs_flash_erase();
    err = nvs_flash_init();
  }
  if (err != ESP_OK) {
    Serial.println("ConfigManager: Failed to initialize NVS!");
    return false;
  }

  // Open NVS namespace
  err = nvs_open(_namespace, NVS_READWRITE, &_nvsHandle);
  if (err != ESP_OK) {
    Serial.println("ConfigManager: Failed to open NVS namespace!");
    return false;
  }

  Serial.println("ConfigManager: Initialized");
  return true;
}

bool ConfigManager::loadConfig(DeviceConfig& config)
{
  if (_nvsHandle == 0) {
    Serial.println("ConfigManager: NVS not initialized!");
    return false;
  }

  // Try to load values
  esp_err_t err = nvs_get_u32(_nvsHandle, "h_high_int", (uint32_t*)&config.humidity_limit_high);
  if (err != ESP_OK && err != ESP_ERR_NVS_NOT_FOUND) {
    Serial.println("ConfigManager: Error reading humidity_limit_high");
    return false;
  }

  err = nvs_get_u32(_nvsHandle, "h_low_int", (uint32_t*)&config.humidity_limit_low);
  if (err != ESP_OK && err != ESP_ERR_NVS_NOT_FOUND) {
    Serial.println("ConfigManager: Error reading humidity_limit_low");
    return false;
  }

  err = nvs_get_u16(_nvsHandle, "quiet_sh", &config.quiet_time_start_hour);
  if (err != ESP_OK && err != ESP_ERR_NVS_NOT_FOUND) {
    Serial.println("ConfigManager: Error reading quiet_time_start_hour");
    return false;
  }

  err = nvs_get_u16(_nvsHandle, "quiet_sm", &config.quiet_time_start_min);
  if (err != ESP_OK && err != ESP_ERR_NVS_NOT_FOUND) {
    Serial.println("ConfigManager: Error reading quiet_time_start_min");
    return false;
  }

  err = nvs_get_u16(_nvsHandle, "quiet_eh", &config.quiet_time_end_hour);
  if (err != ESP_OK && err != ESP_ERR_NVS_NOT_FOUND) {
    Serial.println("ConfigManager: Error reading quiet_time_end_hour");
    return false;
  }

  err = nvs_get_u16(_nvsHandle, "quiet_em", &config.quiet_time_end_min);
  if (err != ESP_OK && err != ESP_ERR_NVS_NOT_FOUND) {
    Serial.println("ConfigManager: Error reading quiet_time_end_min");
    return false;
  }

  err = nvs_get_u16(_nvsHandle, "pir_on_time", &config.pir_relay_on_time);
  if (err != ESP_OK && err != ESP_ERR_NVS_NOT_FOUND) {
    Serial.println("ConfigManager: Error reading pir_relay_on_time");
    return false;
  }

  // If any value was not found, initialize to defaults and save
  if (err == ESP_ERR_NVS_NOT_FOUND) {
    initDefaultConfig(config);
    saveConfig(config);
    Serial.println("ConfigManager: Loaded defaults");
  } else {
    Serial.println("ConfigManager: Loaded config from NVS");
  }

  return true;
}

bool ConfigManager::saveConfig(const DeviceConfig& config)
{
  if (_nvsHandle == 0) {
    Serial.println("ConfigManager: NVS not initialized!");
    return false;
  }

  esp_err_t err = ESP_OK;

  err = nvs_set_u32(_nvsHandle, "h_high_int", *(uint32_t*)&config.humidity_limit_high);
  if (err != ESP_OK) {
    Serial.println("ConfigManager: Error saving humidity_limit_high");
    return false;
  }

  err = nvs_set_u32(_nvsHandle, "h_low_int", *(uint32_t*)&config.humidity_limit_low);
  if (err != ESP_OK) {
    Serial.println("ConfigManager: Error saving humidity_limit_low");
    return false;
  }

  err = nvs_set_u16(_nvsHandle, "quiet_sh", config.quiet_time_start_hour);
  if (err != ESP_OK) {
    Serial.println("ConfigManager: Error saving quiet_time_start_hour");
    return false;
  }

  err = nvs_set_u16(_nvsHandle, "quiet_sm", config.quiet_time_start_min);
  if (err != ESP_OK) {
    Serial.println("ConfigManager: Error saving quiet_time_start_min");
    return false;
  }

  err = nvs_set_u16(_nvsHandle, "quiet_eh", config.quiet_time_end_hour);
  if (err != ESP_OK) {
    Serial.println("ConfigManager: Error saving quiet_time_end_hour");
    return false;
  }

  err = nvs_set_u16(_nvsHandle, "quiet_em", config.quiet_time_end_min);
  if (err != ESP_OK) {
    Serial.println("ConfigManager: Error saving quiet_time_end_min");
    return false;
  }

  err = nvs_set_u16(_nvsHandle, "pir_on_time", config.pir_relay_on_time);
  if (err != ESP_OK) {
    Serial.println("ConfigManager: Error saving pir_relay_on_time");
    return false;
  }

  err = nvs_commit(_nvsHandle);
  if (err != ESP_OK) {
    Serial.println("ConfigManager: Error committing to NVS");
    return false;
  }

  Serial.println("ConfigManager: Configuration saved");
  return true;
}

bool ConfigManager::resetConfig(DeviceConfig& config)
{
  initDefaultConfig(config);
  return saveConfig(config);
}

bool ConfigManager::resetWiFiCredentials()
{
  // Properly disconnect WiFi and turn off module before resetting
  WiFi.disconnect(true);  // Disconnect and turn off persistent WiFi
  WiFi.mode(WIFI_OFF);    // Explicitly turn off WiFi mode
  delay(500);             // Give it time to settle
  
  // Now reset WiFiManager settings
  WiFiManager wm;
  wm.resetSettings();
  Serial.println("ConfigManager: WiFi credentials reset and WiFi module powered off");
  return true;
}

void ConfigManager::initDefaultConfig(DeviceConfig& config)
{
  config.humidity_limit_high = 60.0;
  config.humidity_limit_low = 55.0;
  config.quiet_time_start_hour = 22;
  config.quiet_time_start_min = 0;
  config.quiet_time_end_hour = 6;
  config.quiet_time_end_min = 0;
  config.pir_relay_on_time = 300;
}

ConfigManager::~ConfigManager()
{
  if (_nvsHandle != 0) {
    nvs_close(_nvsHandle);
  }
}
