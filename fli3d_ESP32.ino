/* 
 *  Fli3d - core system functionality
 *   *  
 *  for ESP32 MH-ET LIVE MiniKit board with the following connections:
 *  - serial connection to ESP32CAM at 115200 baud
 *  - serial connection to NEO6MV2 GPS receiver at 57600 baud
 *  - I2C bus to MPU6050/MPU9250 accelerometer/gyroscope and BMP280 pressure sensor 
 *  - separation wire (GND is mated, open is unmated)
 *  - WL102-341 radio transmitter
 *  
 *  use partition scheme "Default with spiffs" or custom partition scheme "Fli3d ESP32 (OTA/maximized SPIFFS)"
 */

// Set versioning
#define SW_VERSION "Fli3d ESP32 v1.1.0 (20220827)"
#define PLATFORM_ESP32 // tell which platform we are on

// Set functionality to compile
//#define RADIO
#define PRESSURE
#define MOTION
#define GPS
#define CAMERA
//#define SERIAL_KEEPALIVE_OVERRIDE

// Libraries
#include "fli3d.h"
#include <ArduinoOTA.h>

// Global variables used in this file
bool reset_gps_timer, separation_sts_changed;
extern char buffer[JSON_MAX_SIZE];

TaskHandle_t LoopCore0;

void setup() {
  // Initialize serial connection to ESP32CAM (or for debug)
  serial_setup ();
  
  // Load default (hardcoded) WiFi and other settings, as fallback
  load_default_config();
  ccsds_init();
 
  // If FS enabled and initialization successful, load WiFi and other settings from configuration files on FS (accessible over FTP)
  sprintf (buffer, "%s started on %s", SW_VERSION, subsystemName[SS_THIS]); 
  if (config_this->fs_enable) {
    if (tm_this->fs_enabled = fs_setup()) {
      sprintf (buffer, "%s started on %s", SW_VERSION, subsystemName[SS_THIS]); 
      publish_event (STS_THIS, SS_THIS, EVENT_INIT, buffer);
      publish_packet ((ccsds_t*)tm_this);  // #0
      if (file_load_settings (FS_LITTLEFS)) {
        file_load_config (FS_LITTLEFS, config_this->config_file); // WiFi and other settings
        file_load_routing (FS_LITTLEFS, config_this->routing_file); // TM routing settings
      }
    }
  }
  else {
    sprintf (buffer, "%s started on %s", SW_VERSION, subsystemName[SS_THIS]); 
    publish_event (STS_THIS, SS_THIS, EVENT_INIT, buffer);
  }
  publish_packet ((ccsds_t*)tm_this);  // #1

  // If WiFi enabled (AP/client), initialize
  if (config_this->wifi_enable) {
    wifi_setup();
    publish_packet ((ccsds_t*)tm_this);  // #2
  } 

  // Initialise subsystems
  #ifdef CAMERA
  if (config_esp32.camera_enable) { 
    esp32.camera_enabled = true;
  }
  #endif // CAMERA 
  
  #ifdef GPS
  if (esp32.gps_enabled = gps_setup()) {
    publish_packet ((ccsds_t*)tm_this);  // #3
  }
  #endif // GPS
  
  #ifdef MOTION
  if (esp32.motion_enabled = motion_setup()) {
    //mpu6050_calibrate();    // TODO: to be done offline on loose sensor, then put calibration values in configuration file
    //mpu6050_checkConfig(); 
    //mpu6050_printConfig(); 
    publish_packet ((ccsds_t*)tm_this);  // #4
  }
  #endif // MOTION
  
  #ifdef PRESSURE
  if (esp32.pressure_enabled = pressure_setup()) { // needs to be after MOTION
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
  if (config_esp32.ftp_enable) {
    esp32.ftp_enabled = ftp_setup();
    publish_packet ((ccsds_t*)tm_this);  // #7
  }

  // Initialize OTA
  if (config_esp32.ota_enable) {
    ota_setup();
  }

  // Initialize time-critical monitoring through core 0
  xTaskCreatePinnedToCore(
    loop_core0,          // name of the task function
    "loop core 0",       // name of the task
    1000,                // memory assigned for the task
    NULL,                // parameter to pass if any
    1,                   // priority of task, starting from 0(Highestpriority) *IMPORTANT*( if set to 1 and there is no activity in your 2nd loop, it will reset the esp32)
    &LoopCore0,          // Reference name of taskHandle variable
    0);                  // choose core (0 or 1)
  
  // Initialize Timer and close initialisation
  ntp_check();
  timer_setup();
  esp32.opsmode = MODE_CHECKOUT;
  publish_event (STS_THIS, SS_THIS, EVENT_INIT, "Initialisation complete");  
}

void loop_core0( void * parameter ) {
  for (;;) {
    // ESP32CAM with OV2640 camera and SD
    #ifdef CAMERA
    if (serial_check()) {
      serial_parse();
    }
    #endif // CAMERA
    delay(1);
  }
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

  // MPU6050 or MPU9250 accelerometer/gyroscope
  #ifdef MOTION
  else if (var_timer.do_motion and esp32.motion_enabled) {
    start_millis = millis();
    if (mpu_acquire()) {
      publish_packet ((ccsds_t*)&motion);
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

  // TC check
  #ifndef ASYNCUDP
  start_millis = millis(); 
  yamcs_tc_check();
  timer_esp32.tc_duration += millis() - start_millis;
  #endif
    
  // Serial keepalive mechanism: if no data received over serial, send out ping and hope for reaction
  #ifndef SERIAL_KEEPALIVE_OVERRIDE
  start_millis = millis();    
  serial_keepalive();
  timer_this->serial_duration += millis() - start_millis;
  #else
  tm_this->serial_connected = true;
  tm_this->warn_serial_connloss = false;
  #endif

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
