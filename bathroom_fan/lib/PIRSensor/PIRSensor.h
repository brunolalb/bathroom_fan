#ifndef PIRSENSOR_H
#define PIRSENSOR_H

#include <Arduino.h>

// PIR Sensor configuration
#define PIR_PIN                    18
#define PIR_IGNORE_AFTER_HOURS     22
#define PIR_IGNORE_AFTER_MIN       0
#define PIR_IGNORE_UNTIL_HOURS     6
#define PIR_IGNORE_UNTIL_MIN       0

class PIRSensor {
public:
  // Constructor
  PIRSensor(uint8_t pin = PIR_PIN);
  
  // Initialize the PIR sensor (attach interrupt)
  void begin();
  
  // Check if movement was detected
  bool isMovementDetected();
  
  // Reset the movement flag
  void resetMovement();
  
  // Static interrupt handler (for FreeRTOS compatibility)
  static void interruptHandler();

private:
  uint8_t _pin;
  static volatile bool _movementFlag;
  static PIRSensor *_instance;
};

#endif
