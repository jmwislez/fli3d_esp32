/* 
 *  Fli3d - core system functionality
 *  
 *  for ESP32 MH-ET LIVE MiniKit board with the following connections:
 *  - serial connection to ESP32CAM at 115200 baud
 *  - serial connection to NEO6MV2 GPS receiver at 57600 baud
 *  - I2C bus to MPU6050 accelerometer/gyroscope and BMP280 pressure sensor 
 *  - separation wire (GND is mated, open is unmated)
 *  - WL-102 radio transmitter
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
#define SW_VERSION "Fli3d core v0.9.0 (20200421)"
#define PLATFORM_ESP32 // tell which platform we are one 

// Set functionality to compile
#define RADIO
#define PRESSURE
#define MOTION
#define GPS
#define ESP32CAM
#define DEBUG_OVER_SERIAL

// Libraries
#include "fli3d.h"
#include <ArduinoOTA.h>

// Global variables used in this file
bool reset_gps_timer, separation_sts_changed;
extern char buffer[TM_MAX_MSG_SIZE], bus_buffer[TM_MAX_MSG_SIZE + 25];

void setup() {
  Serial.begin (SerialBaud);
  Serial.println ();
  Serial.setDebugOutput (true);
  tm_this->mem_free = ESP.getFreeHeap();
  eeprom_setup (); 
  eeprom_load (EEPROM_NETWORK, 0);
  eeprom_load (EEPROM_THIS, sizeof(eeprom_network));
  #ifdef DEBUG_OVER_SERIAL  
  eeprom_this->debug_over_serial = true;
  tm_this->serial_connected = true;
  #endif
  sprintf (buffer, "%s started on %s", SW_VERSION, subsystemName[SS_THIS]); 
  bus_publish_event (STS_THIS, SS_THIS, EVENT_INIT, buffer);
  bus_publish_pkt (TM_THIS);  
  if (eeprom_this->wifi_enable) {
    tm_this->wifi_enabled = wifi_setup ();
    bus_publish_pkt (TM_THIS);  
  } 

  #ifdef ESP32CAM
  if (eeprom_esp32.camera_enable) {
    esp32.camera_enabled = true;
  }
  #endif // ESP32CAM 
  #ifdef GPS
  if (esp32.gps_enabled = gps_setup ()) {
    neo6mv2_checkConfig ();
    if (eeprom_esp32.gps_debug) { neo6mv2_printConfig (); }
    bus_publish_pkt (TM_THIS);  
  }
  #endif // GPS
  #ifdef MOTION
  if (esp32.motion_enabled = motion_setup ()) {
    //mpu6050_calibrate ();    // to be done offline, with vertical still rocket, then hardcode calibration values
    mpu6050_checkConfig (); 
    if (eeprom_esp32.motion_debug) { Serial.println ("MPU6050 status after init ..."); mpu6050_printConfig (); }
    bus_publish_pkt (TM_THIS);  
  }
  #endif // MOTION
  #ifdef PRESSURE
  if (esp32.pressure_enabled = pressure_setup ()) { // needs to be after MOTION
    bmp280_checkConfig ();
    if (eeprom_esp32.pressure_debug) { bmp280_printConfig (); }
    bus_publish_pkt (TM_THIS);  
  }
  #endif // PRESSURE
  #ifdef RADIO
  if (eeprom_esp32.radio_enable) {
    esp32.radio_enabled = radio_setup ();
    bus_publish_pkt (TM_THIS);  
  }
  #endif // RADIO
  separation_setup ();
  timer_setup ();
  esp32.opsmode = MODE_CHECKOUT;
  bus_publish_event (STS_THIS, SS_THIS, EVENT_INIT, "Initialisation complete");  
}

void loop() {
  static uint32_t start_millis;
  static uint16_t data_len;
  // timer
  timer_loop ();
  
  // separation status (monitored via interrupt)
  if (separation_sts_changed) {
    separation_publish ();
    separation_sts_changed = false;
  }
 
  // BMP280 pressure sensor
  #ifdef PRESSURE
  if (timer.do_pressure and esp32.pressure_enabled) {
    if (eeprom_esp32.timer_debug) { start_millis = millis (); }
    if (bmp280_acquire ()) {
      bus_publish_pkt (TM_PRESSURE);
    }
    else {
      // will try to reset pressure sensor once, and then give up
      esp32.pressure_enabled = pressure_setup ();
    }
    timer.do_pressure = false;
    if (eeprom_esp32.timer_debug) { timer.pressure_duration = min((uint32_t)255, (uint32_t)millis() - start_millis); }
  } 
  #endif // PRESSURE

  // MPU6050 accelerometer/gyroscope
  #ifdef MOTION
  if (timer.do_motion and esp32.motion_enabled) {
    if (eeprom_esp32.timer_debug) { start_millis = millis (); }
    if (mpu6050_acquire ()) {
      bus_publish_pkt (TM_MOTION);
    }
    else {
      // will try to reset accelerometer once, and then give up
      esp32.motion_enabled = motion_setup ();
    }
    timer.do_motion = false;
    if (eeprom_esp32.timer_debug) { timer.motion_duration = min((uint32_t)255, (uint32_t)millis() - start_millis); }
  } 
  #endif // MOTION

  // ESP32CAM with OV2640 camera and SD
  #ifdef ESP32CAM
  if (data_len = serial_check ()) {
    if (eeprom_esp32.timer_debug) { start_millis = millis (); }
    serial_parse (data_len);
    if (eeprom_esp32.timer_debug) { timer.esp32cam_duration = min((uint32_t)255, (uint32_t)millis() - start_millis); }
  }
  #endif // ESP32CAM
  
  // NEO6MV2 GPS
  #ifdef GPS
  if (esp32.gps_enabled) {
    if (eeprom_esp32.timer_debug) { start_millis = millis (); }
    if (gps_check ()) {
      bus_publish_pkt (TM_GPS);
      reset_gps_timer = true;
      timer.do_gps = false;
    }
    if (timer.do_gps) {
      bus_publish_pkt (TM_GPS);
      timer.do_gps = false;
    }
    if (eeprom_esp32.timer_debug) { timer.gps_duration = min((uint32_t)255, (uint32_t)millis() - start_millis); }
  }
  #endif // GPS
  
  // WL101-341 RF 433 MHz Transmitter
  #ifdef RADIO
  if (timer.do_radio) {
    if (eeprom_esp32.timer_debug) { start_millis = millis (); }
    bus_publish_pkt (TM_RADIO);
    timer.do_radio = false;
    if (eeprom_esp32.timer_debug) { timer.radio_duration = min((uint32_t)255, (uint32_t)millis() - start_millis); }
  }
  #endif // RADIO

  if (eeprom_esp32.ota_enable) {
    ArduinoOTA.handle();
  }
}
