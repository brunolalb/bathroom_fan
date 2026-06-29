#include "RGBLed.h"

// Constructor
RGBLed::RGBLed(uint8_t redPin, uint8_t greenPin, uint8_t bluePin)
  : _redPin(redPin), _greenPin(greenPin), _bluePin(bluePin) {
}

// Initialize LED pins
void RGBLed::begin() {
  pinMode(_redPin, OUTPUT);
  pinMode(_greenPin, OUTPUT);
  pinMode(_bluePin, OUTPUT);
  allOff();  // Start with all LEDs off
}

// Red LED functions
void RGBLed::redOn() {
  digitalWrite(_redPin, LOW);
}

void RGBLed::redOff() {
  digitalWrite(_redPin, HIGH);
}

bool RGBLed::isRedOn() {
  return !digitalRead(_redPin);
}

// Green LED functions
void RGBLed::greenOn() {
  digitalWrite(_greenPin, LOW);
}

void RGBLed::greenOff() {
  digitalWrite(_greenPin, HIGH);
}

bool RGBLed::isGreenOn() {
  return !digitalRead(_greenPin);
}

// Blue LED functions
void RGBLed::blueOn() {
  digitalWrite(_bluePin, LOW);
}

void RGBLed::blueOff() {
  digitalWrite(_bluePin, HIGH);
}

bool RGBLed::isBlueOn() {
  return !digitalRead(_bluePin);
}

// All LED control
void RGBLed::allOn() {
  redOn();
  greenOn();
  blueOn();
}

void RGBLed::allOff() {
  redOff();
  greenOff();
  blueOff();
}
