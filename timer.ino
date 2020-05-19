/*
 * Fli3d - TM acquisition timer functionality 
 */

void timer_setup () {
    timer.next_second = 1000*(millis()/1000) + 1000;
    timer.radio_interval = (1000 / eeprom_esp32.radio_rate);
    timer.pressure_interval = (1000 / eeprom_esp32.pressure_rate);
    timer.motion_interval = (1000 / eeprom_esp32.motion_rate);
    timer.gps_interval = (1000 / eeprom_esp32.gps_rate);
}

void timer_loop () {
  // timer
  timer.millis = millis ();
  esp32.timer_rate++;
  if (timer.millis >= timer.next_second) {
    bus_publish_pkt (TM_ESP32);
    timer.next_second += 1000;
  }
  else if (timer.millis >= timer.next_radio_time) {
    timer.do_radio = true;
    timer.next_radio_time = timer.millis + timer.radio_interval;
  }
  else if (timer.millis >= timer.next_pressure_time) {
    timer.do_pressure = true;
    timer.next_pressure_time = timer.millis + timer.pressure_interval;
  }
  else if (timer.millis >= timer.next_motion_time) {
    timer.do_motion = true;
    timer.next_motion_time = timer.millis + timer.motion_interval;
  }
  else if (reset_gps_timer) {
    timer.next_gps_time = timer.millis + 1000;  // return to 1Hz when no data any more    
  }
  else if (timer.millis >= timer.next_gps_time) {
    timer.do_gps = true;
    timer.next_gps_time = timer.millis + 1000;  // 1Hz as long as no data 
  }
  if (eeprom_esp32.timer_debug) {
    timer.loop_duration = timer.millis - millis();
    if (timer.loop_duration > 1) {
      bus_publish_pkt (TM_TIMER);
    }
  }
  // Serial keepalive mechanism
  if (!eeprom_this->debug_over_serial and timer.millis - timer.last_serial_out_millis > KEEPALIVE_INTERVAL) {
    if (tm_this->serial_connected) {
      Serial.println ("O");
    }
    else {
      Serial.println ("o");
    }
    timer.last_serial_out_millis = millis();
  }
  if (!eeprom_this->debug_over_serial and tm_this->serial_connected and millis()-timer.last_serial_in_millis > 2*KEEPALIVE_INTERVAL) {
    announce_lost_serial_connection ();
  }
  // Wifi monitoring mechanism
  if (timer.millis - timer.last_wifi_check) {
    if (WiFi.status() != WL_CONNECTED) {
      if (tm_this->wifi_connected) {
        tm_this->wifi_connected = false;
        tm_this->warn_wifi_connloss = true;
      }
    }
    else {
      if (!tm_this->wifi_connected) {
        tm_this->wifi_connected = true;
      }
    }
    timer.last_wifi_check = millis();
  }
}
