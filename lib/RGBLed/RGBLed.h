#ifndef RGBLED_H
#define RGBLED_H

#include <Arduino.h>

class RGBLed {
public:
  // Constructor
  RGBLed(uint8_t redPin, uint8_t greenPin, uint8_t bluePin);
  
  // Initialize LED pins
  void begin();
  
  // Red LED functions
  void redOn();
  void redOff();
  bool isRedOn();
  
  // Green LED functions
  void greenOn();
  void greenOff();
  bool isGreenOn();
  
  // Blue LED functions
  void blueOn();
  void blueOff();
  bool isBlueOn();
  
  // All LED control
  void allOn();
  void allOff();

private:
  uint8_t _redPin;
  uint8_t _greenPin;
  uint8_t _bluePin;
};

#endif
