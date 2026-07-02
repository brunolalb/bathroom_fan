#include "DHTSensor.h"

// Global pointer to instance for static callback
static DHTSensor *_dhtSensorInstance = nullptr;

DHTSensor::DHTSensor(uint8_t pin, uint8_t type)
  : _pin(pin), _type(type), _dht(nullptr), _dhtQueue(nullptr) {
}

void DHTSensor::begin() {
  // Create DHT sensor instance
  _dht = new DHT(_pin, _type);
  _dht->begin();
  
  // Create queue for sensor data
  _dhtQueue = xQueueCreate(5, sizeof(DHTData));
  if (_dhtQueue == NULL) {
    Serial.println("ERROR: Failed to create DHT queue");
    return;
  }
  
  // Store instance pointer for static callback
  _dhtSensorInstance = this;
  
  // Create FreeRTOS task for reading sensor
  xTaskCreate(
    DHTSensor::dhtTaskWrapper,  // Static task function
    "DHT_Sensor",                // Task name
    10000,                        // Stack size
    this,                         // Task parameter (pass this pointer)
    2,                            // Priority
    nullptr                       // Task handle
  );
}

QueueHandle_t DHTSensor::getQueue() {
  return _dhtQueue;
}

// Static wrapper function for FreeRTOS task
void DHTSensor::dhtTaskWrapper(void *param) {
  DHTSensor *instance = (DHTSensor *)param;
  instance->_dhtTask();
  vTaskDelete(nullptr);  // Delete task when done
}

// Instance task function
void DHTSensor::_dhtTask() {
  TickType_t xLastWakeTime = xTaskGetTickCount();
  float humidity_local = 50.0;      // initialized for EXAUSTOR_TEST mode
  float temperature_local = 25.0;   // initialized for EXAUSTOR_TEST mode
  DHTData queue_data;
  char msg[30];
  
  while (1) {
    queue_data.humidity = -1.0;
    queue_data.temperature = -273.0;
    
    /* read DHT11 sensor */
#ifdef EXAUSTOR_TEST
    humidity_local = humidity_local + (float(random(-50, 50)) / 10.0);
    if (humidity_local < 50.0) humidity_local = 50.0;
    if (humidity_local > 65.0) humidity_local = 56.0;
    temperature_local = temperature_local + (float(random(-10, 10)) / 10.0);
    if (temperature_local < 10.0) temperature_local = 25.0;
#else
    humidity_local = _dht->readHumidity();
    temperature_local = _dht->readTemperature();
#endif

    if (isnan(humidity_local)) {
      Serial.println("Failed to read humidity");
    } else if (isnan(temperature_local)) {
      Serial.println("Failed to read temperature");
    } else {
      snprintf(msg, 30, "DHT data: %.1f %%, %.1f C", humidity_local, temperature_local);
      Serial.println(msg);

      queue_data.humidity = humidity_local;
      queue_data.temperature = temperature_local;
      if (_dhtQueue) {
        if (xQueueSend(_dhtQueue, (void *)&queue_data, pdMS_TO_TICKS(100)) != pdPASS) {
          Serial.println("Failed to send data to DHT queue");
        }
      }
    }

    vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(DHT_READ_PERIOD));
  }
}
