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
 */

// Set versioning
#define SW_VERSION "Fli3d ESP32 v0.9.6 (20220724)"
#define PLATFORM_ESP32 // tell which platform we are on

// Set functionality to compile
#define RADIO
#define PRESSURE
#define MOTION
#define GPS
#define ESP32CAM
#define DEBUG_OVER_SERIAL // overrides keep-alive mechanism over serial when ESP32CAM is not present, thus disabling serial buffer and enabling keeping sending of TM even if no response

// Libraries
#include "fli3d.h"
#include <ArduinoOTA.h>

// Global variables used in this file
bool reset_gps_timer, separation_sts_changed;
extern char buffer[JSON_MAX_SIZE];

void setup() {
  // Initialize serial connection to ESP32CAM (or for debug)
  Serial.begin (SerialBaud);
  Serial.println();
  Serial.setDebugOutput (true);
  
  // Boot after ESP32CAM (who's master for TM storage) is ready to receive
  delay (10000);
   
  // Load default (hardcoded) WiFi and other settings, as fallback
  load_default_config();
  ccsds_init();
 
  // If FS enabled and initialization successful, load WiFi and other settings from configuration files on FS (accessible over FTP)
  sprintf (buffer, "%s started on %s", SW_VERSION, subsystemName[SS_THIS]); 
  if (config_this->fs_enable) {
    if (tm_this->fs_enabled = fs_setup()) {
      publish_event (STS_THIS, SS_THIS, EVENT_INIT, buffer);
      publish_packet ((ccsds_t*)tm_this);  // #0
      if (file_load_settings (FS_LITTLEFS)) {
        file_load_config (FS_LITTLEFS, config_this->config_file); // WiFi and other settings
        file_load_routing (FS_LITTLEFS, config_this->routing_file); // TM routing settings
      }
    }
  }
  else {
    publish_event (STS_THIS, SS_THIS, EVENT_INIT, buffer);
  }
  #ifdef DEBUG_OVER_SERIAL  
  config_this->debug_over_serial = true;
  tm_this->serial_connected = true;
  #endif // DEBUG_OVER_SERIAL
  publish_packet ((ccsds_t*)tm_this);  // #1

  // If WiFi enabled (AP/client), initialize
  if (config_this->wifi_enable) {
    wifi_setup();
    publish_packet ((ccsds_t*)tm_this);  // #2
  } 

  // Initialise subsystems
  #ifdef ESP32CAM
  if (config_esp32.camera_enable) { 
    esp32.camera_enabled = true;
  }
  #endif // ESP32CAM 
  #ifdef GPS
  if ((esp32.gps_enabled = gps_setup())) {
    publish_packet ((ccsds_t*)tm_this);  // #3
  }
  #endif // GPS
  #ifdef MOTION
  if ((esp32.motion_enabled = motion_setup())) {
    //mpu6050_calibrate();    // TODO: to be done offline on loose sensor, then put calibration values in configuration file
    mpu6050_checkConfig(); 
    //mpu6050_printConfig(); 
    publish_packet ((ccsds_t*)tm_this);  // #4
  }
  #endif // MOTION
  #ifdef PRESSURE
  if ((esp32.pressure_enabled = pressure_setup())) { // needs to be after MOTION
    bmp280_checkConfig();
    publish_packet ((ccsds_t*)tm_this);  // #5
  }
  #endif // PRESSURE
  #ifdef RADIO
  if (config_esp32.radio_enable) {
    esp32.radio_enabled = radio_setup();
    publish_packet ((ccsds_t*)tm_this);  // #6
  }
  #endif // RADIO
  separation_setup();

  // Initialize FTP server
  if (config_esp32.ftp_enable and esp32.fs_enabled) {
    esp32.ftp_enabled = ftp_setup();
    publish_packet ((ccsds_t*)tm_this);  // #7
  }

  // Initialize Timer and close initialisation
  esp32.opsmode = MODE_CHECKOUT;
  ntp_check();
  timer_setup();
  publish_event (STS_THIS, SS_THIS, EVENT_INIT, "Initialisation complete");  
}

void loop() {
  static uint32_t start_millis;
  
  timer_loop();
  
  // separation status (monitored via interrupt)
  if (separation_sts_changed) {
    separation_publish();
    separation_sts_changed = false;
  }
 
  // BMP280 pressure sensor
  #ifdef PRESSURE
  else if (var_timer.do_pressure and esp32.pressure_enabled) {
    start_millis = millis();
    if (bmp280_acquire()) {
      publish_packet ((ccsds_t*)&bmp280);
    }
    else {
      // will try to reset pressure sensor once, and then give up
      esp32.pressure_enabled = pressure_setup();
    }
    var_timer.do_pressure = false;
    timer_esp32.pressure_duration += millis() - start_millis;
  } 
  #endif // PRESSURE

  // MPU6050 accelerometer/gyroscope
  #ifdef MOTION
  else if (var_timer.do_motion and esp32.motion_enabled) {
    start_millis = millis();
    if (mpu6050_acquire()) {
      publish_packet ((ccsds_t*)&mpu6050);
    }
    else {
      // will try to reset accelerometer once, and then give up
      esp32.motion_enabled = motion_setup();
    }
    var_timer.do_motion = false;
    timer_esp32.motion_duration += millis() - start_millis;
  } 
  #endif // MOTION

  // WL101-341 RF 433 MHz Transmitter
  #ifdef RADIO
  else if (esp32.radio_enabled) {
    if (var_timer.do_radio) {
      start_millis = millis();
      publish_packet ((ccsds_t*)&radio);
      var_timer.do_radio = false;
      timer_esp32.radio_duration += millis() - start_millis;
    }
  }
  #endif // RADIO

  // ESP32CAM with OV2640 camera and SD
  #ifdef ESP32CAM
  if (serial_check()) {
    start_millis = millis();
    serial_parse();
    timer_esp32.esp32cam_duration += millis() - start_millis;
  }
  #endif // ESP32CAM
  
  // NEO6MV2 GPS
  #ifdef GPS
  if (esp32.gps_enabled) {
    start_millis = millis();
    if (config_esp32.gps_udp_raw_enable) {
      publish_udp_gps();
    }
    else {
      if (gps_check()) {
        publish_packet ((ccsds_t*)&neo6mv2);
        reset_gps_timer = true;
        var_timer.do_gps = false;
      }
      if (var_timer.do_gps) {
        neo6mv2.millis = millis();
        publish_packet ((ccsds_t*)&neo6mv2);
        var_timer.do_gps = false;
      }
    }
    timer_esp32.gps_duration += millis() - start_millis;
  }
  #endif // GPS
  
  // Serial keepalive mechanism
  start_millis = millis();    
  if (!config_this->debug_over_serial and timer_esp32.millis - var_timer.last_serial_out_millis > KEEPALIVE_INTERVAL) {
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
  timer_esp32.serial_duration += millis() - start_millis;

  // OTA check
  if (esp32.opsmode == MODE_CHECKOUT and config_esp32.ota_enable) {
    start_millis = millis();    
    ArduinoOTA.handle();
    esp32.ota_enabled = true;
    timer_esp32.ota_duration += millis() - start_millis;
  }
  
  // FTP check
  if ((esp32.opsmode == MODE_CHECKOUT or esp32.opsmode == MODE_DONE) and tm_this->ftp_enabled) {
    // FTP server is active when Fli3d is being prepared or done
    start_millis = millis();    
    ftp_check (config_this->buffer_fs);
    timer_esp32.ftp_duration += millis() - start_millis;
  }

  // TC check
  #ifndef ASYNCUDP
  if (esp32.opsmode == MODE_CHECKOUT or esp32.opsmode == MODE_DONE) {
    start_millis = millis(); 
    // TC are possible when Fli3d is not flying
    yamcs_tc_check();
    timer_esp32.tc_duration += millis() - start_millis;
  }
  #endif

  // wifi check
  if (var_timer.do_wifi and tm_this->wifi_enabled) {
    start_millis = millis();
    wifi_check();
    timer_esp32.wifi_duration += millis() - start_millis;
  }

  // NTP check
  if (!tm_this->time_set and esp32.opsmode == MODE_CHECKOUT and var_timer.do_ntp and esp32.wifi_enabled) {
    start_millis = millis();
    ntp_check();
    timer_esp32.wifi_duration += millis() - start_millis;
  }
}
