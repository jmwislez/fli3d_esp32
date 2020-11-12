/*
 * Fli3d - radio functionality
 */

#ifdef RADIO
#include <RH_ASK.h>

RH_ASK radio_tx (RADIO_BAUD, DUMMY_PIN1, RF433_TX_PIN, DUMMY_PIN2);

bool radio_setup () {
  delay(1000);
  if (radio_tx.init()) {
    publish_event (STS_ESP32, SS_RADIO, EVENT_INIT, "Radio transmitter initialized");
    return true;
  }
  else {
    publish_event (STS_ESP32, SS_RADIO, EVENT_ERROR, "Failed to initialize radio transmitter");
    return false; 
  }
}

void publish_radio () {
  if (esp32.radio_enabled) {
    radio_tx.send((uint8_t *)&radio, sizeof(tm_radio_t));
  }
}

#endif // RADIO
