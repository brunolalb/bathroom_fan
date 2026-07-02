#ifndef SWITCH_SENSOR_H
#define SWITCH_SENSOR_H

#include <Arduino.h>

class SwitchSensor {
public:
  /**
   * Constructor - Initializes SwitchSensor
   * @param pin The GPIO pin connected to the switch
   * @param debounceTime Debounce time in milliseconds
   */
  SwitchSensor(uint8_t pin = 22, unsigned long debounceTime = 500);

  /**
   * Initialize the switch sensor
   * Configures pin and attaches interrupt handler
   */
  void begin();

  /**
   * Check if switch was changed (pressed/released)
   * @return true if switch changed, false otherwise
   */
  bool isSwitched();

  /**
   * Reset the switch changed flag
   */
  void resetSwitch();

  /**
   * Destructor
   */
  ~SwitchSensor();

private:
  uint8_t _pin;
  unsigned long _debounceTime;
  volatile bool _switchFlag;
  unsigned long _lastInterruptTime;
  
  // Static data for interrupt handler
  static SwitchSensor *_instance;
  static void IRAM_ATTR interruptHandler();
};

#endif
