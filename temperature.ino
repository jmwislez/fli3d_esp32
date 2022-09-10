/*
 * Fli3d - temperature sensor functionality
 */

#ifdef TEMPERATURE

#include <DHTesp.h>

DHTesp dht;

bool temperature_setup() {
  dht.setup(DHT_PIN, DHTesp::DHT22);
  if (dht.getStatus()) {
    publish_event (STS_ESP32, SS_ESP32, EVENT_ERROR, "Temperature sensor not initialized");
    return false;
  }
  else {
    sprintf(buffer, "%s temperature sensor initialized", dhtName[dht.getModel()]);
    publish_event (STS_ESP32, SS_SEPARATION, EVENT_INIT, buffer);
    return true;
  }
}

void temperature_acquire() {
  esp32.temperature = (int16_t)(dht.getTemperature()*100);
}

#endif // TEMPERATURE
