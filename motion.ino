// TODO: test mpu6050_checkConfig
// TODO: test impact of loss or temporary interruption of communications
// TODO: limit floating point calculations?
// TODO: configuration different in cool or hot start
// TODO: check by testing if indeed 8g (as stated in the manual) is the right setting for calibration, or if lowest sensitivity (2g) is better
// TODO: data from motion sensor is trash

/*
 * Fli3d - Accelerometer/Gyroscope functionality (poll driven)
 */

#ifdef MOTION

#include "I2Cdev.h"
#include "MPU6050.h"
#include "Wire.h"

// Settings and global variables for MPU6050

#define ACCEL_RANGE          3 // values: 0: +/-2g, 1: +/-4g, 2: +/-8g, 3: +/-16g // TODO: set to 2 or 3?
#define GYRO_RANGE           2 // values: 0: +/-250deg/s, 1: +/-500deg/s, 2: +/-1000deg/s, 3: +/-2000deg/s  // TODO: leave at 0?
#define SELFTEST_ACCEL_RANGE 0 // should be 2 according to manual
#define SELFTEST_GYRO_RANGE  0 // should be 0 according to manual
#define CALIB_ACCEL_RANGE    0 // should be 0
#define CALIB_GYRO_RANGE     0 // should be 0
#define ACCEL_BUFFER_SIZE    50
#define GYRO_BUFFER_SIZE     50

const float gravity_constant = 9.81156f;
float mpu6050_accel_scaleFactor[4] { 16384.0, 8192.0, 4096.0, 2048.0 };
float mpu6050_gyro_scaleFactor[4] { 131.0, 65.5, 32.8, 16.4 };
uint8_t mpu6050_ratioFactor[4] { 1, 2, 4, 8 };

/*
const struct {
  int16_t x_sensor = -256;
  int16_t y_sensor = -395;
  int16_t z_sensor = 2068;
} mpu6050_accel_offset; // outcome from calibration, using native sensor axes, to be done offline with still vertical rocket

const struct {
  int16_t x_sensor = 224;
  int16_t y_sensor = -84;
  int16_t z_sensor = 74;
} mpu6050_gyro_offset; // outcome from calibration, using native sensor axes, to be done offline with still rocket
*/
const struct {
  int16_t x_sensor = 0;
  int16_t y_sensor = 0;
  int16_t z_sensor = 0;
} mpu6050_accel_offset; // compensate built-in offset

const struct {
  int16_t x_sensor = 0;
  int16_t y_sensor = 0;
  int16_t z_sensor = 0;
} mpu6050_gyro_offset; // compensate built-in offset


MPU6050 mpu;

bool motion_setup () {
  return (mpu6050_setup());
}

bool mpu6050_setup () {
  // Set up I2C bus
  Wire.begin (I2C_SDA_PIN, I2C_SCL_PIN); 
  Wire.setClock (400000);
  mpu.initialize ();

  // Check if we find a properly responding MPU6050 device
  if (!mpu6050_testConnection ()) {
    return false;
  }
  
  // Do self-test now, as it changes the settings
  mpu6050_selfTest (); 

  // Configure MPU6050  // done with direct writes, as there is something wrong with i2cdev on ESP32 and I don't want to debug
  uint8_t dlpf_bw;
  if (config_esp32.motion_rate < 20) {
    dlpf_bw = MPU6050_DLPF_BW_5;
  }
  else if (config_esp32.motion_rate < 40) {
    dlpf_bw = MPU6050_DLPF_BW_10;
  }
  else if (config_esp32.motion_rate < 84) {
    dlpf_bw = MPU6050_DLPF_BW_20;
  }
  else if (config_esp32.motion_rate < 196) {
    dlpf_bw = MPU6050_DLPF_BW_42;
  }
  else if (config_esp32.motion_rate < 376) {
    dlpf_bw = MPU6050_DLPF_BW_98;
  }
  else if (config_esp32.motion_rate < 512) {
    dlpf_bw = MPU6050_DLPF_BW_188;
  }
  else {
    dlpf_bw = MPU6050_DLPF_BW_256;
  } 
  mpu.setXAccelOffset (mpu6050_accel_offset.x_sensor);
  mpu.setYAccelOffset (mpu6050_accel_offset.y_sensor);
  mpu.setZAccelOffset (mpu6050_accel_offset.z_sensor);
  mpu.setXGyroOffset (mpu6050_gyro_offset.x_sensor);
  mpu.setYGyroOffset (mpu6050_gyro_offset.y_sensor);
  mpu.setZGyroOffset (mpu6050_gyro_offset.z_sensor);
  mpu.setRate((uint8_t)((1000 - config_esp32.motion_rate) / config_esp32.motion_rate)); // TODO: is it meaningful to set the rate in case of polling?
  mpu.setDLPFMode(dlpf_bw);
  mpu.setFullScaleGyroRange(GYRO_RANGE);
  mpu.setFullScaleAccelRange(ACCEL_RANGE);
  mpu.setI2CBypassEnabled (true);
  mpu.setI2CMasterModeEnabled(false);
  mpu.setClockSource(MPU6050_CLOCK_PLL_XGYRO);
  mpu.setSleepEnabled(false);
  publish_event (STS_ESP32, SS_MPU6050, EVENT_INIT, "Initialized MPU6050 accelerometer/gyroscope sensor");
  return true;
}

bool mpu6050_testConnection () {
  if (mpu.testConnection()) {
    publish_event (STS_ESP32, SS_MPU6050, EVENT_INIT, "Found MPU6050 6DOF motion sensor");
    return true;
  }
  else {
    publish_event (STS_ESP32, SS_MPU6050, EVENT_ERROR, "Failed to connect to MPU6050; disabling");
    return false;
  } 
}

void mpu6050_calibrate () {
  // Configure MPU6050 gyro and accelerometer for bias calculation
  I2Cdev::writeByte(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_CONFIG, 0x01);                    // Set low-pass filter to 188 Hz
  I2Cdev::writeByte(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_SMPLRT_DIV, 0x00);                // Set sample rate to 1 kHz
  I2Cdev::writeByte(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_GYRO_CONFIG, CALIB_GYRO_RANGE);   // Set gyro full-scale (should be 250 degrees per second, maximum sensitivity)
  I2Cdev::writeByte(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_ACCEL_CONFIG, CALIB_ACCEL_RANGE); // Set accelerometer full-scale (should be 2 g, maximum sensitivity)
  mpu.CalibrateGyro();
  sprintf (buffer, "MPU6050 accelerometer calibration completed (offsets:[X:%d,Y:%d,Z:%d])", mpu.getXAccelOffset(), mpu.getYAccelOffset(), mpu.getZAccelOffset());
  publish_event (STS_ESP32, SS_MPU6050, EVENT_INIT, buffer);
  mpu.CalibrateAccel(); // has been modified for gravity vector along x axis, with amplitude as parameter
  sprintf (buffer, "MPU6050 gyroscope calibration completed (offsets:[X:%d,Y:%d,Z:%d])", mpu.getXGyroOffset(), mpu.getYGyroOffset(), mpu.getZGyroOffset());
  publish_event (STS_ESP32, SS_MPU6050, EVENT_INIT, buffer);
  publish_event (STS_ESP32, SS_MPU6050, EVENT_WARNING, "Hardcode MPU6050 calibration parameters and disable calibration!");
}  

bool mpu6050_selfTest() {
  // Should return percent deviation from factory trim values, +/- 14 or less deviation is a pass
  uint8_t rawData[4];
  uint8_t selfTest[6];
  float factoryTrim[6];
  float result[6];
  // Configure the accelerometer for self-test
  I2Cdev::writeByte (MPU6050_DEFAULT_ADDRESS, MPU6050_RA_ACCEL_CONFIG, 0xF0); // Enable self test on all three axes and set accelerometer range to +/- 8 g (needed for calibration)
  I2Cdev::writeByte (MPU6050_DEFAULT_ADDRESS, MPU6050_RA_GYRO_CONFIG,  0xE0); // Enable self test on all three axes and set gyro range to +/- 250 degrees/s (needed for calibration)
  delay(250);  // Delay a while to let the device execute the self-test
  I2Cdev::readByte (MPU6050_DEFAULT_ADDRESS, MPU6050_RA_SELF_TEST_X, &rawData[0], 0); // X-axis self-test results
  I2Cdev::readByte (MPU6050_DEFAULT_ADDRESS, MPU6050_RA_SELF_TEST_Y, &rawData[1], 0); // Y-axis self-test results
  I2Cdev::readByte (MPU6050_DEFAULT_ADDRESS, MPU6050_RA_SELF_TEST_Z, &rawData[2], 0); // Z-axis self-test results
  I2Cdev::readByte (MPU6050_DEFAULT_ADDRESS, MPU6050_RA_SELF_TEST_A, &rawData[3], 0); // Mixed-axis self-test results
  // Extract the acceleration test results first
  selfTest[0] = (rawData[0] >> 3) | (rawData[3] & 0x30) >> 4 ; // XA_TEST result is a five-bit unsigned integer
  selfTest[1] = (rawData[1] >> 3) | (rawData[3] & 0x0C) >> 2 ; // YA_TEST result is a five-bit unsigned integer
  selfTest[2] = (rawData[2] >> 3) | (rawData[3] & 0x03) >> 0 ; // ZA_TEST result is a five-bit unsigned integer
  // Extract the gyration test results first
  selfTest[3] = rawData[0]  & 0x1F ; // XG_TEST result is a five-bit unsigned integer
  selfTest[4] = rawData[1]  & 0x1F ; // YG_TEST result is a five-bit unsigned integer
  selfTest[5] = rawData[2]  & 0x1F ; // ZG_TEST result is a five-bit unsigned integer   
  // Process results to allow final comparison with factory set values
  factoryTrim[0] = (mpu6050_accel_scaleFactor[SELFTEST_ACCEL_RANGE]*0.34)*(pow((0.92/0.34), (((float)selfTest[0] - 1.0)/30.0))); // FT[Xa] factory trim calculation
  factoryTrim[1] = (mpu6050_accel_scaleFactor[SELFTEST_ACCEL_RANGE]*0.34)*(pow((0.92/0.34), (((float)selfTest[1] - 1.0)/30.0))); // FT[Ya] factory trim calculation
  factoryTrim[2] = (mpu6050_accel_scaleFactor[SELFTEST_ACCEL_RANGE]*0.34)*(pow((0.92/0.34), (((float)selfTest[2] - 1.0)/30.0))); // FT[Za] factory trim calculation
  factoryTrim[3] = ( 25.0*mpu6050_gyro_scaleFactor[SELFTEST_GYRO_RANGE])*(pow(1.046, ((float)selfTest[3] - 1.0) ));              // FT[Xg] factory trim calculation
  factoryTrim[4] = (-25.0*mpu6050_gyro_scaleFactor[SELFTEST_GYRO_RANGE])*(pow(1.046, ((float)selfTest[4] - 1.0) ));              // FT[Yg] factory trim calculation
  factoryTrim[5] = ( 25.0*mpu6050_gyro_scaleFactor[SELFTEST_GYRO_RANGE])*(pow(1.046, ((float)selfTest[5] - 1.0) ));              // FT[Zg] factory trim calculation

  // Report results as a ratio of (STR - FT)/FT; the change from Factory Trim of the Self-Test Response
  // To get to percent, must multiply by 100 and subtract result from 100
  for (int i = 0; i < 6; i++) {
    result[i] = 100.0 + 100.0*((float)selfTest[i] - factoryTrim[i])/factoryTrim[i]; // Report percent differences
  }  
  if(result[0] < 1.0f && result[1] < 1.0f && result[2] < 1.0f && result[3] < 1.0f && result[4] < 1.0f && result[5] < 1.0f) {
    sprintf (buffer, "MPU6050 passed self-test (A_dev\%:[X:%.2f,Y:%.2f,Z:%.2f],G_dev\%:[X:%.2f,Y:%.2f,Z:%.2f])", result[0], result[1], result[2], result[3], result[4], result[5]);
    publish_event (STS_ESP32, SS_MPU6050, EVENT_INIT, buffer);
    return true;  
  }
  else {
    sprintf (buffer, "MPU6050 failed self-test (A_dev\%:[X:%.2f,Y:%.2f,Z:%.2f],G_dev\%:[X:%.2f,Y:%.2f,Z:%.2f])", result[0], result[1], result[2], result[3], result[4], result[5]);
    publish_event (STS_ESP32, SS_MPU6050, EVENT_WARNING, buffer);
    return false;    
  }
}    

bool mpu6050_acquire () {
  static int16_t a_raw_x_rocket, a_raw_y_rocket, a_raw_z_rocket, g_raw_x_rocket, g_raw_y_rocket, g_raw_z_rocket;
  static float mpu6050_accel_xy, mpu6050_accel_xy2; 

  if (mpu.testConnection ()) {
    esp32.motion_active = true;
    mpu6050.millis = millis();
    mpu.getMotion6(&a_raw_z_rocket, &a_raw_x_rocket, &a_raw_y_rocket, &g_raw_z_rocket, &g_raw_x_rocket, &g_raw_y_rocket);  // axes changed for rocket orientation: x_sensor=z_rocket, y_sensor=x_rocket, z_sensor=y_rocket 
    mpu6050.accel_x_rocket = (float)(a_raw_x_rocket - mpu6050.a_x_rocket_offset/mpu6050_ratioFactor[ACCEL_RANGE])/mpu6050_accel_scaleFactor[ACCEL_RANGE];
    mpu6050.accel_y_rocket = (float)(a_raw_y_rocket - mpu6050.a_y_rocket_offset/mpu6050_ratioFactor[ACCEL_RANGE])/mpu6050_accel_scaleFactor[ACCEL_RANGE];
    mpu6050.accel_z_rocket = (float)(a_raw_z_rocket - mpu6050.a_z_rocket_offset/mpu6050_ratioFactor[ACCEL_RANGE])/mpu6050_accel_scaleFactor[ACCEL_RANGE];
    mpu6050.gyro_x_rocket = (float)(g_raw_x_rocket - mpu6050.g_x_rocket_offset/mpu6050_ratioFactor[GYRO_RANGE])/mpu6050_gyro_scaleFactor[GYRO_RANGE];
    mpu6050.gyro_y_rocket = (float)(g_raw_y_rocket - mpu6050.g_y_rocket_offset/mpu6050_ratioFactor[GYRO_RANGE])/mpu6050_gyro_scaleFactor[GYRO_RANGE];
    mpu6050.gyro_z_rocket = (float)(g_raw_z_rocket - mpu6050.g_z_rocket_offset/mpu6050_ratioFactor[GYRO_RANGE])/mpu6050_gyro_scaleFactor[GYRO_RANGE];
    mpu6050_accel_xy2 = mpu6050.accel_x_rocket*mpu6050.accel_x_rocket + mpu6050.accel_y_rocket*mpu6050.accel_y_rocket;
    mpu6050_accel_xy = sqrt (mpu6050_accel_xy2);
    mpu6050.tilt = 180.0/M_PI * atan2 (mpu6050_accel_xy, sqrt (gravity_constant*gravity_constant-mpu6050_accel_xy2));
    mpu6050.a = mpu6050.accel_z_rocket-gravity_constant*cos(M_PI*mpu6050.tilt/180.0);
    mpu6050.g = sqrt (mpu6050_accel_xy2 + mpu6050.accel_z_rocket*mpu6050.accel_z_rocket) / gravity_constant;
    mpu6050.rpm = mpu6050.gyro_z_rocket/6.0;
    if (esp32.opsmode == MODE_CHECKOUT) {
      //mpu6050_zero_accel (a_raw_x_rocket, a_raw_y_rocket, a_raw_z_rocket);
      mpu6050_zero_gyro (g_raw_x_rocket, g_raw_y_rocket, g_raw_z_rocket);
    }
    else {
      if (mpu6050.g < 0.95) {
        if (esp32.opsmode != MODE_FREEFALL) {
          publish_event (STS_ESP32, SS_MPU6050, EVENT_INFO, "Start of free-fall detected");
          esp32.opsmode = MODE_FREEFALL;
        }
      }
      else if (0.95 < mpu6050.g < 1.05) { // TODO: determine appropriate limits
        if (esp32.opsmode != MODE_STATIC) {
          publish_event (STS_ESP32, SS_MPU6050, EVENT_INFO, "Static rocket detected");
          esp32.opsmode = MODE_STATIC;
        }
      }
      else {
        if (esp32.opsmode != MODE_THRUST) {
          publish_event (STS_ESP32, SS_MPU6050, EVENT_INFO, "Start of thrust detected");
          esp32.opsmode = MODE_THRUST;
        }
      }
    }
    return true;
  }
  else {
    // lost connection, attempt reset
    publish_event (STS_ESP32, SS_MPU6050, EVENT_WARNING, "Lost connection to MPU6050; attempting reset");
    return false;
  }
}

void mpu6050_zero_gyro (int16_t new_gyro_x, int16_t new_gyro_y, int16_t new_gyro_z) {
  static int16_t gyro_x_buffer[GYRO_BUFFER_SIZE];
  static int16_t gyro_y_buffer[GYRO_BUFFER_SIZE];
  static int16_t gyro_z_buffer[GYRO_BUFFER_SIZE];
  static int32_t gyro_x_sum, gyro_y_sum, gyro_z_sum;
  static uint8_t gyro_buffer_pos;
  static bool gyro_zero_level_ok;
  gyro_x_sum = gyro_x_sum - gyro_x_buffer[gyro_buffer_pos] + new_gyro_x;
  gyro_y_sum = gyro_y_sum - gyro_y_buffer[gyro_buffer_pos] + new_gyro_y;
  gyro_z_sum = gyro_z_sum - gyro_z_buffer[gyro_buffer_pos] + new_gyro_z;
  gyro_x_buffer[gyro_buffer_pos] = new_gyro_x;
  gyro_y_buffer[gyro_buffer_pos] = new_gyro_y;
  gyro_z_buffer[gyro_buffer_pos] = new_gyro_z;
  if (++gyro_buffer_pos == GYRO_BUFFER_SIZE) {
    gyro_buffer_pos = 0;
    gyro_zero_level_ok = true;
  }
  if (gyro_zero_level_ok) {
    mpu6050.g_z_rocket_offset = gyro_x_sum / GYRO_BUFFER_SIZE;
    mpu6050.g_x_rocket_offset = gyro_y_sum / GYRO_BUFFER_SIZE;
    mpu6050.g_y_rocket_offset = gyro_z_sum / GYRO_BUFFER_SIZE;
  }
}

void mpu6050_zero_accel (int16_t new_accel_x, int16_t new_accel_y, int16_t new_accel_z) {
  static int16_t accel_x_buffer[ACCEL_BUFFER_SIZE];
  static int16_t accel_y_buffer[ACCEL_BUFFER_SIZE];
  static int16_t accel_z_buffer[ACCEL_BUFFER_SIZE];
  static int32_t accel_x_sum, accel_y_sum, accel_z_sum;
  static uint8_t accel_buffer_pos;
  static bool accel_zero_level_ok;
  accel_x_sum = accel_x_sum - accel_x_buffer[accel_buffer_pos] + new_accel_x;
  accel_y_sum = accel_y_sum - accel_y_buffer[accel_buffer_pos] + new_accel_y;
  accel_z_sum = accel_z_sum - accel_z_buffer[accel_buffer_pos] + new_accel_z;
  accel_x_buffer[accel_buffer_pos] = new_accel_x;
  accel_y_buffer[accel_buffer_pos] = new_accel_y;
  accel_z_buffer[accel_buffer_pos] = new_accel_z;
  if (++accel_buffer_pos == ACCEL_BUFFER_SIZE) {
    accel_buffer_pos = 0;
    accel_zero_level_ok = true;
  }
  if (accel_zero_level_ok) {
    mpu6050.a_z_rocket_offset = accel_x_sum / ACCEL_BUFFER_SIZE;
    mpu6050.a_x_rocket_offset = accel_y_sum / ACCEL_BUFFER_SIZE;
    mpu6050.a_y_rocket_offset = accel_z_sum / ACCEL_BUFFER_SIZE;
  }
}

void motion_imageSmear () { // TODO: when to send, to be most effective?  on demand?
  // provides to camera the current expected image smear, in deg/s
  sprintf (buffer, "{\"cmd\":\"set_smear\",\"trans\":%.2f,\"axial\":%.2f}", sqrt (mpu6050.gyro_y_rocket*mpu6050.gyro_y_rocket+mpu6050.gyro_z_rocket*mpu6050.gyro_z_rocket), mpu6050.gyro_x_rocket);
  publish_event (TC_ESP32CAM, SS_OV2640, EVENT_CMD, buffer);
}

bool mpu6050_checkConfig () {
  // 0x00 - 0x05: yes
  // 0x06 - 0x0C: yes (offsets)
  // 0x0D - 0x10: no (self-test outputs)
  // 0x11 - 0x12: ??? (not defined in map)
  // 0x13 - 0x18: yes (offsets)
  // 0x19 - 0x1C: yes
  // 0x1D - 0x22: ??? (not defined in map)
  // 0x23 - 0x24: yes
  // 0x25 - 0x35: maybe (slave configurations, unused)
  // 0x36 - 0x38: yes
  // 0x39       : ??? (not defined in map)
  // 0x3A       : yes
  // 0x3B - 0x60: no (measurements)
  // 0x61 - 0x62: ??? (not defined in map)
  // 0x63 - 0x66: no (slave I/O)
  // 0x67       : maybe (slave configurations, unused)
  // 0x68       : no (W only, reset)
  // 0x69       : ??? (not defined in map)
  // 0x6A - 0x6C: yes
  // 0x6D - 0x71: ??? (not defined in map)
  // 0x72 - 0x74: no (measurements)
  // 0x75       : no (R only, device ID)
  #define start_address0  0x00
  #define end_address0    0x0C
  #define start_address1  0x11
  #define end_address1    0x3A
  #define start_address2  0x67
  #define end_address2    0x71
  char mpu6050_config0_reference[2*(end_address0-start_address0+1)+1] = "8303813f41aeff00fe75081428";
  char mpu6050_config0[2*(end_address0-start_address0+1)+1];
  char mpu6050_config1_reference[2*(end_address1-start_address1+1)+1] = "6306e0f000000000000000000000000000000000000000000000000000000200";
  char mpu6050_config1[2*(end_address1-start_address1+1)+1];
  char mpu6050_config2_reference[2*(end_address2-start_address2+1)+1] = "000000";
  char mpu6050_config2[2*(end_address2-start_address2+1)+1];

  if (mpu6050_checkMemory (start_address0, end_address0, mpu6050_config0, mpu6050_config0_reference) &&
      mpu6050_checkMemory (start_address1, end_address1, mpu6050_config1, mpu6050_config1_reference) &&
      mpu6050_checkMemory (start_address2, end_address2, mpu6050_config2, mpu6050_config2_reference)) {
    publish_event (STS_ESP32, SS_MPU6050, EVENT_INIT, "MPU6050 configuration checked");
  }
}

bool mpu6050_checkMemory (uint8_t start_address, uint8_t end_address, char *hexstring, char *hexstring_reference) {  
  uint8_t data;
  for (uint8_t ctr=0; ctr<(end_address-start_address+1); ctr++) {
    I2Cdev::readByte(MPU6050_DEFAULT_ADDRESS, start_address+ctr, &data);
    sprintf (&hexstring[2*ctr], "%02x", data);
  }
  if (strcmp(hexstring, hexstring_reference)) {
    sprintf (buffer, "MPU6050 configuration in %02x - %02x block not as expected (%s)", start_address, end_address, hexstring);
    publish_event (STS_ESP32, SS_MPU6050, EVENT_WARNING, buffer);
    return false;
  }
  else {
    return true;
  }
}

void mpu6050_printConfig () {
  uint16_t FullScaleGyroRange[4] = {250, 500, 1000, 2000};
  uint8_t FullScaleAccelRange[4] = {2, 4, 8, 16};
  Serial.println ("===== MPU6050 status =======================================");
  Serial.print ("Rate:                         ");
  Serial.print (1000/(1+mpu.getRate()));
  Serial.println (" Hz");
  Serial.print ("DLFPMode:                     ");
  switch (mpu.getDLPFMode()) {
    case 0x00: Serial.println ("MPU6050_DLPF_BW_256"); break;
    case 0x01: Serial.println ("MPU6050_DLPF_BW_188"); break;
    case 0x02: Serial.println ("MPU6050_DLPF_BW_98"); break;
    case 0x03: Serial.println ("MPU6050_DLPF_BW_42"); break;
    case 0x04: Serial.println ("MPU6050_DLPF_BW_20"); break;
    case 0x05: Serial.println ("MPU6050_DLPF_BW_10"); break;
    case 0x06: Serial.println ("MPU6050_DLPF_BW_5"); break;
  }
  Serial.print ("FullScaleGyroRange:           +/- ");
  Serial.print (FullScaleGyroRange[mpu.getFullScaleGyroRange()]);
  Serial.println (" deg/s");
  Serial.print ("FullScaleAccelRange:          +/- ");
  Serial.print (FullScaleAccelRange[mpu.getFullScaleAccelRange()]);
  Serial.println (" g");
  Serial.print ("DHPFMode:                     ");
  switch (mpu.getDHPFMode()) {
    case 0x00: Serial.println ("MPU6050_DHPF_RESET"); break;
    case 0x01: Serial.println ("MPU6050_DHPF_5"); break;
    case 0x02: Serial.println ("MPU6050_DHPF_2P5"); break;
    case 0x03: Serial.println ("MPU6050_DHPF_1P25"); break;
    case 0x04: Serial.println ("MPU6050_DHPF_0P63"); break;
    case 0x07: Serial.println ("MPU6050_DHPF_HOLD"); break;
  }
  Serial.print ("FreefallDetectionThreshold:   ");
  Serial.println (mpu.getFreefallDetectionThreshold());
  Serial.print ("FreefallDetectionDuration:    ");
  Serial.println (mpu.getFreefallDetectionDuration());
  Serial.print ("MotionDetectionThreshold:     ");
  Serial.println (mpu.getMotionDetectionThreshold());
  Serial.print ("MotionDetectionDuration:      ");
  Serial.println (mpu.getMotionDetectionDuration());
  Serial.print ("ZeroMotionDetectionThreshold: ");
  Serial.println (mpu.getZeroMotionDetectionThreshold());
  Serial.print ("ZeroMotionDetectionDuration:  ");
  Serial.println (mpu.getZeroMotionDetectionDuration());
  Serial.print ("TempFIFOEnabled:              ");
  Serial.println (mpu.getTempFIFOEnabled());
  Serial.print ("XGyroFIFOEnabled:             ");
  Serial.println (mpu.getXGyroFIFOEnabled());
  Serial.print ("YGyroFIFOEnabled:             ");
  Serial.println (mpu.getYGyroFIFOEnabled());
  Serial.print ("ZGyroFIFOEnabled:             ");
  Serial.println (mpu.getZGyroFIFOEnabled());
  Serial.print ("AccelFIFOEnabled:             ");
  Serial.println (mpu.getAccelFIFOEnabled());
  Serial.print ("MasterClockSpeed:             ");
  Serial.println (mpu.getMasterClockSpeed());
  Serial.print ("PassthroughStatus:            ");
  Serial.println (mpu.getPassthroughStatus());
  Serial.print ("InterruptMode:                ");
  Serial.println (mpu.getInterruptMode());
  Serial.print ("InterruptDrive:               ");
  Serial.println (mpu.getInterruptDrive());
  Serial.print ("InterruptLatch:               ");
  Serial.println (mpu.getInterruptLatch());
  Serial.print ("InterruptLatchClear:          ");
  Serial.println (mpu.getInterruptLatchClear());
  Serial.print ("I2CBypassEnabled:             ");
  Serial.println (mpu.getI2CBypassEnabled());
  Serial.print ("ClockOutputEnabled:           ");
  Serial.println (mpu.getClockOutputEnabled());
  Serial.print ("IntEnabled:                   ");
  Serial.println (mpu.getIntEnabled());
  Serial.print ("IntFreefallEnabled:           ");
  Serial.println (mpu.getIntFreefallEnabled());
  Serial.print ("IntMotionEnabled:             ");
  Serial.println (mpu.getIntMotionEnabled());
  Serial.print ("IntZeroMotionEnabled:         ");
  Serial.println (mpu.getIntZeroMotionEnabled());
  Serial.print ("IntFIFOBufferOverflowEnabled: ");
  Serial.println (mpu.getIntFIFOBufferOverflowEnabled());
  Serial.print ("IntI2CMasterEnabled:          ");
  Serial.println (mpu.getIntI2CMasterEnabled());
  Serial.print ("IntDataReadyEnabled:          ");
  Serial.println (mpu.getIntDataReadyEnabled());
  Serial.print ("FIFOEnabled:                  ");
  Serial.println (mpu.getFIFOEnabled());
  Serial.print ("I2CMasterModeEnabled:         ");
  Serial.println (mpu.getI2CMasterModeEnabled());
  Serial.print ("SleepEnabled:                 ");
  Serial.println (mpu.getSleepEnabled());
  Serial.print ("WakeCycleEnabled:             ");
  Serial.println (mpu.getWakeCycleEnabled());
  Serial.print ("TempSensorEnabled:            ");
  Serial.println (mpu.getTempSensorEnabled());
  Serial.print ("ClockSource:                  ");
  Serial.println (mpu.getClockSource());
  Serial.print ("DeviceID:                     ");
  Serial.println (mpu.getDeviceID());
  Serial.print ("OTPBankValid:                 ");
  Serial.println (mpu.getOTPBankValid());
  Serial.print ("X_gyro_offsetTC:              ");
  Serial.println (mpu.getXGyroOffsetTC());
  Serial.print ("Y_gyro_offsetTC:              ");
  Serial.println (mpu.getYGyroOffsetTC());
  Serial.print ("Z_gyro_offsetTC:              ");
  Serial.println (mpu.getZGyroOffsetTC());
  Serial.print ("XFineGain:                    ");
  Serial.println (mpu.getXFineGain());
  Serial.print ("YFineGain:                    ");
  Serial.println (mpu.getYFineGain());
  Serial.print ("ZFineGain:                    ");
  Serial.println (mpu.getZFineGain());
  Serial.print ("X_accel_offset:               ");
  Serial.println (mpu.getXAccelOffset());
  Serial.print ("Y_accel_offset:               ");
  Serial.println (mpu.getYAccelOffset());
  Serial.print ("Z_accel_offset:               ");
  Serial.println (mpu.getZAccelOffset());
  Serial.print ("X_gyro_offset:                ");
  Serial.println (mpu.getXGyroOffset());
  Serial.print ("Y_gyro_offset:                ");
  Serial.println (mpu.getYGyroOffset());
  Serial.print ("Z_gyro_offset:                ");
  Serial.println (mpu.getZGyroOffset());
  Serial.print ("IntPLLReadyEnabled:           ");
  Serial.println (mpu.getIntPLLReadyEnabled());
  Serial.print ("IntDMPEnabled:                ");
  Serial.println (mpu.getIntDMPEnabled());
  Serial.print ("DMPEnabled:                   ");
  Serial.println (mpu.getDMPEnabled());
  Serial.print ("DMPConfig1:                   ");
  Serial.println (mpu.getDMPConfig1());
  Serial.print ("DMPConfig2:                   ");
  Serial.println (mpu.getDMPConfig2());
  Serial.println ("============================================================");
}

#endif
