#include "PIRSensor.h"

// Static member initialization
volatile bool PIRSensor::_movementFlag = false;
PIRSensor *PIRSensor::_instance = nullptr;

PIRSensor::PIRSensor(uint8_t pin)
  : _pin(pin) {
}

void PIRSensor::begin() {
  // Store instance pointer for static interrupt handler
  _instance = this;
  _movementFlag = false;
  
  // Setup the PIR sensor pin
  pinMode(_pin, INPUT_PULLUP);
  
  // Attach interrupt handler
  attachInterrupt(digitalPinToInterrupt(_pin), PIRSensor::interruptHandler, RISING);
}

bool PIRSensor::isMovementDetected() {
  return _movementFlag;
}

void PIRSensor::resetMovement() {
  _movementFlag = false;
}

// Static interrupt handler
void IRAM_ATTR PIRSensor::interruptHandler() {
  if (_instance != nullptr) {
    _instance->_movementFlag = true;
  }
}
