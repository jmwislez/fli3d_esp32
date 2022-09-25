/*
 * Fli3d - temperature sensor functionality
 */

#ifdef TEMPERATURE

#include <DHTesp.h>

DHTesp dht;

bool temperature_setup() {
  dht.setup(DHT_PIN, DHTesp::AUTO_DETECT);
  if (dht.getStatus()) {
    sprintf(buffer, "Temperature sensor not initialized (%s)", (dht.getStatus()==1)?"TIMEOUT":"CHECKSUM");
    publish_event (STS_ESP32, SS_ESP32, EVENT_ERROR, buffer);
    return false;
  }
  else {
    sprintf(buffer, "%s temperature sensor initialized (%.1f degC)", dhtName[dht.getModel()], dht.getTemperature());
    publish_event (STS_ESP32, SS_SEPARATION, EVENT_INIT, buffer);
    return true;
  }
}

void temperature_acquire() {
  esp32.temperature = int16_t(100*dht.getTemperature());
}

#endif // TEMPERATURE
