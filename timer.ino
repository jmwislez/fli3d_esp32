/*
 * Fli3d - TM acquisition timer functionality 
 */

void timer_setup () {
    var_timer.next_second = 1000*(millis()/1000) + 1000;
    var_timer.radio_interval = (1000 / config_esp32.radio_rate);
    var_timer.pressure_interval = (1000 / config_esp32.pressure_rate);
    var_timer.motion_interval = (1000 / config_esp32.motion_rate);
    var_timer.gps_interval = (1000 / config_esp32.gps_rate);
}

void timer_loop () {
  // data acquisition and publication timer
  timer.millis = millis ();
  if (timer.millis >= var_timer.next_second) {
    bus_publish_pkt (TM_ESP32);
    bus_publish_pkt (TM_TIMER);
    var_timer.next_second += 1000;
  }
  else if (timer.millis >= var_timer.next_radio_time) {
    var_timer.do_radio = true;
    var_timer.next_radio_time = timer.millis + var_timer.radio_interval;
  }
  else if (timer.millis >= var_timer.next_pressure_time) {
    var_timer.do_pressure = true;
    var_timer.next_pressure_time = timer.millis + var_timer.pressure_interval;
  }
  else if (timer.millis >= var_timer.next_motion_time) {
    var_timer.do_motion = true;
    var_timer.next_motion_time = timer.millis + var_timer.motion_interval;
  }
  else if (reset_gps_timer) {
    var_timer.next_gps_time = timer.millis + 1000;  // return to 1Hz when no data any more    
  }
  else if (timer.millis >= var_timer.next_gps_time) {
    var_timer.do_gps = true;
    var_timer.next_gps_time = timer.millis + 1000;  // 1Hz as long as no data 
  }
  else if (timer.millis >= var_timer.next_wifi_time) {
    var_timer.do_wifi = true;
    var_timer.next_wifi_time = timer.millis + WIFI_POLL;
  } 
  else if (timer.millis >= var_timer.next_ntp_time) {
    var_timer.do_ntp = true;
    var_timer.next_ntp_time = timer.millis + 1000;
  }
}
