/* 
 *  Fli3d - core system functionality
 *  
 *  for ESP32 MH-ET LIVE MiniKit board with the following connections:
 *  - serial connection to ESP32CAM at 115200 baud
 *  - serial connection to NEO6MV2 GPS receiver at 57600 baud
 *  - I2C bus to MPU6050 accelerometer/gyroscope and BMP280 pressure sensor 
 *  - separation wire (GND is mated, open is unmated)
 *  - WL102-341 radio transmitter
 *  
 *  Functionality:
 *  - Configure and acquire data from accelerometer/gyroscope sensor 
 *  - Configure and acquire data from pressure/temperature sensor 
 *  - Configure and acquire data from gps receiver 
 *  - Acquire status information on separation status 
 *  - Acquire status information and commands from ESP32CAM 
 *  - Provide status information of all subsystems to ESP32CAM 
 *  - Compile telemetry packets 
 *  - Transmit telemetry packets over 433.92 MHz radio transmitter and over wifi
 *  
 */

// Set versioning
#define SW_VERSION "Fli3d ESP32 v0.9.2 (20201022)"
#define PLATFORM_ESP32 // tell which platform we are one 

// Set functionality to compile
#define RADIO
#define PRESSURE
#define MOTION
#define GPS
#define ESP32CAM
#define DEBUG_OVER_SERIAL // override keep-alive mechanism over serial when ESP32CAM is not present

// Libraries
#include "fli3d.h"
#include <ArduinoOTA.h>

// Global variables used in this file
bool reset_gps_timer, separation_sts_changed;
extern char buffer[JSON_MAX_SIZE];

void setup() {
  Serial.begin (SerialBaud);
  Serial.println ();
  Serial.setDebugOutput (true);
  tm_this->mem_free = ESP.getFreeHeap();
  load_default_config ();
  if (config_esp32.fs_enable) {
    esp32.fs_enabled = fs_setup ();
    fs_load_settings ();
    fs_load_config (config_this->config_file);
    fs_load_routing (config_this->routing_file);
  }
  #ifdef DEBUG_OVER_SERIAL  
  config_this->debug_over_serial = true;
  tm_this->serial_connected = true;
  #endif // DEBUG_OVER_SERIAL
  sprintf (buffer, "%s started on %s", SW_VERSION, subsystemName[SS_THIS]); 
  publish_event (STS_THIS, SS_THIS, EVENT_INIT, buffer);
  publish_packet (TM_THIS);  
  if (config_this->wifi_enable) {
    tm_this->wifi_enabled = wifi_setup ();
    publish_packet (TM_THIS);  
  } 

  #ifdef ESP32CAM
  if (config_esp32.camera_enable) {
    esp32.camera_enabled = true;
  }
  #endif // ESP32CAM 
  #ifdef GPS
  if (esp32.gps_enabled = gps_setup ()) {
    neo6mv2_checkConfig ();
    publish_packet (TM_THIS);  
  }
  #endif // GPS
  #ifdef MOTION
  if (esp32.motion_enabled = motion_setup ()) {
    //mpu6050_calibrate ();    // to be done offline, with vertical still rocket, then hardcode calibration values
    mpu6050_checkConfig (); 
    publish_packet (TM_THIS);  
  }
  #endif // MOTION
  #ifdef PRESSURE
  if (esp32.pressure_enabled = pressure_setup ()) { // needs to be after MOTION
    bmp280_checkConfig ();
    publish_packet (TM_THIS);  
  }
  #endif // PRESSURE
  #ifdef RADIO
  if (config_esp32.radio_enable) {
    esp32.radio_enabled = radio_setup ();
    publish_packet (TM_THIS);  
  }
  #endif // RADIO
  separation_setup ();
  if (esp32.fs_enabled) {
    ftp_setup ();
  }
  esp32.opsmode = MODE_CHECKOUT;
  timer_setup ();
  publish_event (STS_THIS, SS_THIS, EVENT_INIT, "Initialisation complete");  
}

void loop() {
  static uint32_t start_millis;
  
  timer_loop ();
  
  // separation status (monitored via interrupt)
  if (separation_sts_changed) {
    separation_publish ();
    separation_sts_changed = false;
  }
 
  // BMP280 pressure sensor
  #ifdef PRESSURE
  if (var_timer.do_pressure and esp32.pressure_enabled) {
    start_millis = millis ();
    if (bmp280_acquire ()) {
      publish_packet (TM_PRESSURE);
    }
    else {
      // will try to reset pressure sensor once, and then give up
      esp32.pressure_enabled = pressure_setup ();
    }
    var_timer.do_pressure = false;
    timer.pressure_duration += millis() - start_millis;
  } 
  #endif // PRESSURE

  // MPU6050 accelerometer/gyroscope
  #ifdef MOTION
  if (var_timer.do_motion and esp32.motion_enabled) {
    start_millis = millis ();
    if (mpu6050_acquire ()) {
      publish_packet (TM_MOTION);
    }
    else {
      // will try to reset accelerometer once, and then give up
      esp32.motion_enabled = motion_setup ();
    }
    var_timer.do_motion = false;
    timer.motion_duration += millis() - start_millis;
  } 
  #endif // MOTION

  // ESP32CAM with OV2640 camera and SD
  #ifdef ESP32CAM
  if (serial_check ()) {
    start_millis = millis ();
    serial_parse ();
    timer.esp32cam_duration += millis() - start_millis;
  }
  #endif // ESP32CAM
  
  // NEO6MV2 GPS
  #ifdef GPS
  if (esp32.gps_enabled) {
    start_millis = millis ();
    if (gps_check ()) {
      publish_packet (TM_GPS);
      reset_gps_timer = true;
      var_timer.do_gps = false;
    }
    if (var_timer.do_gps) {
      publish_packet (TM_GPS);
      var_timer.do_gps = false;
    }
    timer.gps_duration += millis() - start_millis;
  }
  #endif // GPS
  
  // WL101-341 RF 433 MHz Transmitter
  #ifdef RADIO
  if (esp32.radio_enabled) {
    if (var_timer.do_radio) {
      start_millis = millis ();
      publish_packet (TM_RADIO);
      var_timer.do_radio = false;
      timer.radio_duration += millis() - start_millis;
    }
  }
  #endif // RADIO

  // Serial keepalive mechanism
  start_millis = millis ();    
  if (!config_this->debug_over_serial and timer.millis - var_timer.last_serial_out_millis > KEEPALIVE_INTERVAL) {
    if (tm_this->serial_connected) {
      Serial.println ("O");
    }
    else {
      Serial.println ("o");
    }
    var_timer.last_serial_out_millis = millis();
  }
  if (!config_this->debug_over_serial and tm_this->serial_connected and millis()-var_timer.last_serial_in_millis > 2*KEEPALIVE_INTERVAL) {
    tm_this->serial_connected = false;
    tm_this->warn_serial_connloss = true;
  }
  timer.serial_duration += millis() - start_millis;

  // OTA check
  if (config_esp32.ota_enable) {
    start_millis = millis ();    
    ArduinoOTA.handle();
    esp32.ota_enabled = true;
    timer.ota_duration += millis() - start_millis;
  }
  
  // FTP check
  if (esp32.opsmode == MODE_CHECKOUT or esp32.opsmode == MODE_STATIC) {
    // FTP server is active when Fli3d is not
    start_millis = millis ();    
    ftp_check ();
    timer.ftp_duration += millis() - start_millis;
  }

  // TC check
  #ifndef ASYNCUDP
  if (esp32.opsmode == MODE_CHECKOUT or esp32.opsmode == MODE_READY or esp32.opsmode == MODE_STATIC) {
    start_millis = millis (); 
    // TC are possible when Fli3d is not flying
    yamcs_tc_check ();
    timer.tc_duration += millis() - start_millis;
  }
  #endif

  // wifi check
  if (var_timer.do_wifi and esp32.wifi_enabled) {
    start_millis = millis ();
    wifi_check ();
    timer.wifi_duration += millis() - start_millis;
  }

  // activity timer management
  if (var_timer.do_ntp and esp32.wifi_enabled) {
    start_millis = millis ();
    time_check ();
    timer.wifi_duration += millis() - start_millis;
  }
}
