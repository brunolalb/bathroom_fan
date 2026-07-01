#ifndef HARDWARE_CONFIG_H
#define HARDWARE_CONFIG_H

/****************************************
 * WiFi Configuration
 ****************************************/
#ifdef EXAUSTOR_TEST
#define WIFI_HOSTNAME "exaustor_test"
#else
#define WIFI_HOSTNAME "exaustor"
#endif
#define WIFI_AP_NAME "exaustor-setup"
#define NTP_SERVER "pool.ntp.org"

/****************************************
 * RGB LED Configuration
 ****************************************/
#define LED_PIN_RED    2    // Red LED pin
#define LED_PIN_GREEN  5    // Green LED pin
#define LED_PIN_BLUE   21   // Blue LED pin

/****************************************
 * DHT Sensor Configuration
 ****************************************/
#define DHTPIN    19    // DHT11 data pin
#define DHTTYPE   DHT11 // DHT sensor type

/****************************************
 * Relay Configuration
 ****************************************/
#define RELAY_PIN     4          // Relay control pin
#define RELAY_ON()    digitalWrite(RELAY_PIN, LOW)   // Active low
#define RELAY_OFF()   digitalWrite(RELAY_PIN, HIGH)  // Inactive high
#define RELAY_IS_ON() !digitalRead(RELAY_PIN)        // Check if relay is on

/****************************************
 * Switch Configuration
 ****************************************/
#define SWITCH_ENABLE             0      // Enable/disable manual switch
#define SWITCH_PIN                22     // Switch input pin
#define SWITCH_IS_ON()            digitalRead(SWITCH_PIN)  // Check switch state
#define SWITCH_DEBOUNCE_TIME_MS   500    // Debounce time in milliseconds

/****************************************
 * PIR Motion Sensor Configuration
 ****************************************/
#define PIR_PIN                18    // PIR sensor data pin
#define PIR_IGNORE_AFTER_HOURS 22    // Start time to ignore PIR (24-hour format)
#define PIR_IGNORE_AFTER_MIN   0     // Start minute
#define PIR_IGNORE_UNTIL_HOURS 6     // End time to ignore PIR (24-hour format)
#define PIR_IGNORE_UNTIL_MIN   0     // End minute
#define PIR_IGNORE_AFTER       (PIR_IGNORE_AFTER_HOURS * 60 + PIR_IGNORE_AFTER_MIN)
#define PIR_IGNORE_UNTIL       (PIR_IGNORE_UNTIL_HOURS * 60 + PIR_IGNORE_UNTIL_MIN)

/****************************************
 * Application Control Configuration
 ****************************************/
#define CONTROL_LOOP_PERIOD_MS       1000  // Main control loop period in milliseconds

#define RELAY_ON_TIME_SEC_DEFAULT    300   // Default relay ON time (seconds)
#define RELAY_ON_TIME_SEC_HUMIDITY   60    // Relay ON time when humidity triggers it
#define RELAY_ON_TIME_SEC_MIN        30    // Minimum relay ON time
#define RELAY_KEEP_OFF_TIME_SEC_DEFAULT 120 // How long to keep relay OFF after timeout

/****************************************
 * Default Device Configuration
 ****************************************/
#define HUMIDITY_LIMIT_HIGH_DEFAULT   60.0  // Humidity level to trigger relay ON
#define HUMIDITY_LIMIT_LOW_DEFAULT    55.0  // Humidity level to trigger relay OFF
#define QUIET_TIME_START_HOUR_DEFAULT 22   // Quiet hours start (24h format)
#define QUIET_TIME_START_MIN_DEFAULT  0    // Quiet hours start minutes
#define QUIET_TIME_END_HOUR_DEFAULT   6    // Quiet hours end (24h format)
#define QUIET_TIME_END_MIN_DEFAULT    0    // Quiet hours end minutes
#define PIR_RELAY_ON_TIME_DEFAULT     300  // PIR relay on time in seconds

#endif
