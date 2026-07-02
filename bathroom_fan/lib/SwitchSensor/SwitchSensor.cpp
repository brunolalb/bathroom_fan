#include "SwitchSensor.h"

// Static member initialization
SwitchSensor *SwitchSensor::_instance = nullptr;

SwitchSensor::SwitchSensor(uint8_t pin, unsigned long debounceTime)
  : _pin(pin),
    _debounceTime(debounceTime),
    _switchFlag(false),
    _lastInterruptTime(0)
{
  _instance = this;
}

void SwitchSensor::begin()
{
  // Configure pin as input with pull-down
  pinMode(_pin, INPUT_PULLDOWN);
  
  // Attach interrupt handler
  attachInterrupt(digitalPinToInterrupt(_pin), interruptHandler, CHANGE);
  
  Serial.println("SwitchSensor: Initialized");
}

bool SwitchSensor::isSwitched()
{
  return _switchFlag;
}

void SwitchSensor::resetSwitch()
{
  _switchFlag = false;
}

void IRAM_ATTR SwitchSensor::interruptHandler()
{
  if (_instance) {
    unsigned long irq_time = millis();
    
    // Debounce check
    if ((irq_time - _instance->_lastInterruptTime) < _instance->_debounceTime) {
      return;
    }
    
    _instance->_lastInterruptTime = irq_time;
    _instance->_switchFlag = true;
  }
}

SwitchSensor::~SwitchSensor()
{
  // Detach interrupt if needed
  detachInterrupt(digitalPinToInterrupt(_pin));
}
