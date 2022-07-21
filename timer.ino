/*
 * Fli3d - TM acquisition timer functionality 
 */

extern bool sync_fs_ccsds ();
 
void timer_setup () {
    var_timer.next_second = 1000*(millis()/1000) + 1000;
    var_timer.radio_interval = (1000 / config_esp32.radio_rate);
    var_timer.pressure_interval = (1000 / config_esp32.pressure_rate);
    var_timer.motion_interval = (1000 / config_esp32.motion_rate);
    var_timer.gps_interval = (1000 / config_esp32.gps_rate);
}

void timer_loop () {
  // data acquisition and publication timer
  timer_esp32.millis = millis ();
  if (timer_esp32.millis >= var_timer.next_second) {
    publish_packet ((ccsds_t*)&esp32);
    publish_packet ((ccsds_t*)&timer_esp32);
    sync_fs_ccsds ();
    var_timer.next_second += 1000;
  }
  if (esp32.opsmode != MODE_DONE) {
    if (timer_esp32.millis >= var_timer.next_radio_time) {
      var_timer.do_radio = true;
      var_timer.next_radio_time = timer_esp32.millis + var_timer.radio_interval;
    }
    if (timer_esp32.millis >= var_timer.next_pressure_time) {
      var_timer.do_pressure = true;
      var_timer.next_pressure_time = timer_esp32.millis + var_timer.pressure_interval;
    }
    if (timer_esp32.millis >= var_timer.next_motion_time) {
      var_timer.do_motion = true;
      var_timer.next_motion_time = timer_esp32.millis + var_timer.motion_interval;
    }
    if (reset_gps_timer) {
      var_timer.next_gps_time = timer_esp32.millis + 1000;  // return to 1Hz when no data any more    
    }
    if (timer_esp32.millis >= var_timer.next_gps_time) {
      var_timer.do_gps = true;
      var_timer.next_gps_time = timer_esp32.millis + 1000;  // 1Hz as long as no data 
    }
  }
  if (timer_esp32.millis >= var_timer.next_wifi_time) {
    var_timer.do_wifi = true;
    var_timer.next_wifi_time = timer_esp32.millis + WIFI_CHECK*1000;
  } 
  if (timer_esp32.millis >= var_timer.next_ntp_time) {
    var_timer.do_ntp = true;
    var_timer.next_ntp_time = timer_esp32.millis + NTP_CHECK*1000;
  }
}
