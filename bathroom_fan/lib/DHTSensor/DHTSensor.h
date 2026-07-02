#ifndef DHTSENSOR_H
#define DHTSENSOR_H

#include <Arduino.h>
#include <DHT.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

// DHT sensor configuration
#define DHT_PIN           19
#define DHT_TYPE          DHT11
#define DHT_READ_PERIOD   10000  // milliseconds (10 seconds)

// Data structure for DHT readings
typedef struct {
  float humidity;
  float temperature;
} DHTData;

class DHTSensor {
public:
  // Constructor
  DHTSensor(uint8_t pin = DHT_PIN, uint8_t type = DHT_TYPE);
  
  // Initialize the sensor and create the reading task
  void begin();
  
  // Get the queue handle (for receiving data from the reading task)
  QueueHandle_t getQueue();
  
  // Static task function (called by FreeRTOS)
  static void dhtTaskWrapper(void *param);

private:
  DHT *_dht;
  QueueHandle_t _dhtQueue;
  uint8_t _pin;
  uint8_t _type;
  
  // Instance task function
  void _dhtTask();
};

#endif
