
/*
 * Fli3d - separation detection functionality
 */

void ICACHE_RAM_ATTR separation_detectChange();

void separation_setup() {
  pinMode(SEP_STS_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(SEP_STS_PIN), separation_detectChange, CHANGE);
  publish_event (STS_ESP32, SS_SEPARATION, EVENT_INIT, "Separation detection initialized");
}

void separation_publish() {
  if (esp32.separation_sts) {
    publish_event (STS_ESP32, SS_SEPARATION, EVENT_INFO, "Fli3d separated from rocket");   
  }
  else {
    publish_event (STS_ESP32, SS_SEPARATION, EVENT_INFO, "Fli3d mated to rocket");
  }
}

void separation_detectChange() {
  esp32.separation_sts = digitalRead (SEP_STS_PIN);
  separation_sts_changed = true;
}
