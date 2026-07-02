#ifndef CONFIG_MANAGER_H
#define CONFIG_MANAGER_H

#include <cstdint>
#include <nvs_flash.h>
#include <nvs.h>

struct DeviceConfig {
  float humidity_limit_high;
  float humidity_limit_low;
  uint16_t quiet_time_start_hour;    // 0-23
  uint16_t quiet_time_start_min;     // 0-59
  uint16_t quiet_time_end_hour;      // 0-23
  uint16_t quiet_time_end_min;       // 0-59
  uint16_t pir_relay_on_time;        // seconds relay stays ON after motion
  char timezone_posix[128];          // POSIX timezone string (e.g., "CET-1CEST,M3.5.0/2,M10.5.0/3")
};

class ConfigManager {
public:
  /**
   * Constructor - Initializes ConfigManager
   */
  ConfigManager();

  /**
   * Initialize NVS storage and load configuration
   * @return true if successful, false otherwise
   */
  bool begin();

  /**
   * Load configuration from NVS
   * @param config Reference to config struct to populate
   * @return true if successful, false otherwise
   */
  bool loadConfig(DeviceConfig& config);

  /**
   * Save configuration to NVS
   * @param config Reference to config struct to save
   * @return true if successful, false otherwise
   */
  bool saveConfig(const DeviceConfig& config);

  /**
   * Reset all configuration to defaults
   * @param config Reference to config struct to reset
   * @return true if successful, false otherwise
   */
  bool resetConfig(DeviceConfig& config);

  /**
   * Reset WiFi credentials (erase WiFi Manager data)
   * @return true if successful, false otherwise
   */
  bool resetWiFiCredentials();

  /**
   * Destructor
   */
  ~ConfigManager();

private:
  nvs_handle_t _nvsHandle;
  const char *_namespace;
  
  void initDefaultConfig(DeviceConfig& config);
};

#endif
