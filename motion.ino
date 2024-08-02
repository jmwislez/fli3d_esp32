/*
 * Fli3d - Accelerometer/Gyroscope functionality
 */

#define MPU_6050
//#define MPU_9250

#ifdef MOTION

#include "I2Cdev.h"
#include "Wire.h"
#ifdef MPU_6050
#include "MPU6050.h"
MPU6050 mpu;
const char* mpu_sensor = "MPU6050";
#define CLOCK_PLL_XGYRO MPU6050_CLOCK_PLL_XGYRO
#define DLPF_BW_5 MPU6050_DLPF_BW_5
#define DLPF_BW_10 MPU6050_DLPF_BW_10
#define DLPF_BW_20 MPU6050_DLPF_BW_20
#define DLPF_BW_42 MPU6050_DLPF_BW_42
#define DLPF_BW_98 MPU6050_DLPF_BW_98
#define DLPF_BW_188 MPU6050_DLPF_BW_188
#define DLPF_BW_256 MPU6050_DLPF_BW_256
#define DEFAULT_ADDRESS MPU6050_DEFAULT_ADDRESS
#define RA_ACCEL_CONFIG MPU6050_RA_ACCEL_CONFIG
#define RA_GYRO_CONFIG MPU6050_RA_GYRO_CONFIG
#define RA_SELF_TEST_X MPU6050_RA_SELF_TEST_X
#define RA_SELF_TEST_Y MPU6050_RA_SELF_TEST_Y
#define RA_SELF_TEST_Z MPU6050_RA_SELF_TEST_Z
#define RA_SELF_TEST_A MPU6050_RA_SELF_TEST_A
#endif

#ifdef MPU_9250
#include "MPU9250.h"
#define CLOCK_PLL_XGYRO MPU9250_CLOCK_PLL_XGYRO
#define DLPF_BW_5 MPU9250_DLPF_BW_5
#define DLPF_BW_10 MPU9250_DLPF_BW_10
#define DLPF_BW_20 MPU9250_DLPF_BW_20
#define DLPF_BW_42 MPU9250_DLPF_BW_42
#define DLPF_BW_98 MPU9250_DLPF_BW_98
#define DLPF_BW_188 MPU9250_DLPF_BW_188
#define DLPF_BW_256 MPU9250_DLPF_BW_256
#define DEFAULT_ADDRESS MPU9250_DEFAULT_ADDRESS
#define RA_ACCEL_CONFIG MPU9250_RA_ACCEL_CONFIG
#define RA_GYRO_CONFIG MPU9250_RA_GYRO_CONFIG
#define RA_SELF_TEST_X MPU9250_RA_SELF_TEST_X
#define RA_SELF_TEST_Y MPU9250_RA_SELF_TEST_Y
#define RA_SELF_TEST_Z MPU9250_RA_SELF_TEST_Z
#define RA_SELF_TEST_A MPU9250_RA_SELF_TEST_A
MPU9250 mpu;
const char* mpu_sensor = "MPU9250";
#endif

// Settings and global variables

#define SELFTEST_ACCEL_RANGE 0
#define SELFTEST_GYRO_RANGE  0
#define ACCEL_BUFFER_SIZE    50
#define GYRO_BUFFER_SIZE     50

const int16_t gravity_constant = 981; // [cm/s2]
const int16_t mpu_accel_scaleFactor[4] = { 16384, 8192, 4096, 2048 }; 
const int16_t mpu_gyro_scaleFactor[4] = { 1310, 655, 328, 164 }; // multiplied by 10 to allow for int calculations
const int16_t mpu_magn_scaleFactor[4] = { 1, 1, 1, 1 };
const int8_t  mpu_ratioFactor[4] = { 1, 2, 4, 8 };


bool motion_setup() {
  return (mpu_setup());
}

bool mpu_setup() {
  // Set up I2C bus
  Wire.begin (I2C_SDA_PIN, I2C_SCL_PIN); 
  Wire.setClock (400000);
  // Check if we find a properly responding MPU device
  if (!mpu_testConnection()) {
    return false;
  }
  // Do self-test
  mpu_selfTest(); 
  mpu.reset();
  delay(500);
  // Configure the MPU for use
  mpu.setI2CBypassEnabled (true);
  delay (200);
  mpu.setI2CMasterModeEnabled(false);
  delay (200);
  mpu.setClockSource(CLOCK_PLL_XGYRO);
  delay (200);
  mpu.setSleepEnabled(false);
  delay (200);
  motion_set_samplerate (config_esp32.motion_rate);
  delay (200);
  mpu.setFullScaleGyroRange(motion.gyro_range);
  delay (200);
  mpu.setFullScaleAccelRange(motion.accel_range);
  delay (200);
  #ifdef MPU_9250
  I2Cdev::writeByte (DEFAULT_ADDRESS, 0x0A, 0x01); // enable the magnetometer
  #endif
  delay (200);
  //mpu.setXAccelOffset(0); // TODO: after fully understanding MPU6050, use this to set offset (likely allows use of full range)
  //delay (200);
  //mpu.setYAccelOffset(0); // TODO: after fully understanding MPU6050, use this to set offset (likely allows use of full range)
  //delay (200);
  //mpu.setZAccelOffset(0); // TODO: after fully understanding MPU6050, use this to set offset (likely allows use of full range)
  //delay (200);
  sprintf(buffer, "Initialized %s motion sensor", mpu_sensor);
  publish_event (STS_ESP32, SS_MOTION, EVENT_INIT, buffer);
  return true;
}

void motion_set_samplerate (uint8_t rate) { // TODO: rate is limited to 255, while sensor can do more
  uint8_t dlpf_bw;
  config_esp32.motion_rate = rate;
  if (config_esp32.motion_rate < 20) {
    dlpf_bw = DLPF_BW_5;
  }
  else if (config_esp32.motion_rate < 40) {
    dlpf_bw = DLPF_BW_10;
  }
  else if (config_esp32.motion_rate < 84) {
    dlpf_bw = DLPF_BW_20;
  }
  else if (config_esp32.motion_rate < 196) {
    dlpf_bw = DLPF_BW_42;
  }
  else if (config_esp32.motion_rate < 376) { // TODO: always true
    dlpf_bw = DLPF_BW_98;
  }
  else if (config_esp32.motion_rate < 512) { // TODO: always true
    dlpf_bw = DLPF_BW_188;
  }
  else {
    dlpf_bw = DLPF_BW_256;
  } 
  mpu.setRate((uint8_t)((1000 - config_esp32.motion_rate) / config_esp32.motion_rate)); // TODO: is it meaningful to set the rate in case of polling?
  mpu.setDLPFMode(dlpf_bw);
}

void mpu_set_accel_range (uint8_t accel_range) {
  mpu.setFullScaleAccelRange(accel_range);  
  motion.accel_range = mpu.getFullScaleAccelRange();
}

void mpu_set_gyro_range (uint8_t gyro_range) {
  mpu.setFullScaleGyroRange(gyro_range);
  motion.gyro_range = mpu.getFullScaleGyroRange();
}

bool mpu_testConnection() {
  if (mpu.testConnection()) {
    sprintf(buffer, "Found %s motion sensor", mpu_sensor);
    publish_event (STS_ESP32, SS_MOTION, EVENT_INIT, buffer);
    return true;
  }
  else {
    sprintf(buffer, "Failed to connect to %s motion sensor (deviceID: %u)", mpu_sensor, mpu.getDeviceID());
    publish_event (STS_ESP32, SS_MOTION, EVENT_ERROR, buffer);
    return false;
  } 
}

bool mpu_selfTest() {
  #ifdef MPU_6050
  // Should return percent deviation from factory trim values, +/- 14 or less deviation is a pass
  uint8_t rawData[4];
  uint8_t selfTest[6];
  float factoryTrim[6];
  float result[6];
  // Configure the accelerometer for self-test
  I2Cdev::writeByte (DEFAULT_ADDRESS, RA_ACCEL_CONFIG, 0xF0); // Enable self test on all three axes and set accelerometer range to +/- 8 g (needed for calibration)
  I2Cdev::writeByte (DEFAULT_ADDRESS, RA_GYRO_CONFIG,  0xE0); // Enable self test on all three axes and set gyro range to +/- 250 degrees/s (needed for calibration)
  delay(250);  // Delay a while to let the device execute the self-test
  I2Cdev::readByte (DEFAULT_ADDRESS, RA_SELF_TEST_X, &rawData[0], 0); // X-axis self-test results
  I2Cdev::readByte (DEFAULT_ADDRESS, RA_SELF_TEST_Y, &rawData[1], 0); // Y-axis self-test results
  I2Cdev::readByte (DEFAULT_ADDRESS, RA_SELF_TEST_Z, &rawData[2], 0); // Z-axis self-test results
  I2Cdev::readByte (DEFAULT_ADDRESS, RA_SELF_TEST_A, &rawData[3], 0); // Mixed-axis self-test results
  // Extract the acceleration test results first
  selfTest[0] = (rawData[0] >> 3) | (rawData[3] & 0x30) >> 4 ; // XA_TEST result is a five-bit charinteger
  selfTest[1] = (rawData[1] >> 3) | (rawData[3] & 0x0C) >> 2 ; // YA_TEST result is a five-bit charinteger
  selfTest[2] = (rawData[2] >> 3) | (rawData[3] & 0x03) >> 0 ; // ZA_TEST result is a five-bit charinteger
  // Extract the gyration test results first
  selfTest[3] = rawData[0]  & 0x1F ; // XG_TEST result is a five-bit charinteger
  selfTest[4] = rawData[1]  & 0x1F ; // YG_TEST result is a five-bit charinteger
  selfTest[5] = rawData[2]  & 0x1F ; // ZG_TEST result is a five-bit charinteger   
  // Process results to allow final comparison with factory set values
  factoryTrim[0] = (mpu_accel_scaleFactor[SELFTEST_ACCEL_RANGE]*0.34)*(pow((0.92/0.34), (((float)selfTest[0] - 1.0)/30.0))); // FT[Xa] factory trim calculation
  factoryTrim[1] = (mpu_accel_scaleFactor[SELFTEST_ACCEL_RANGE]*0.34)*(pow((0.92/0.34), (((float)selfTest[1] - 1.0)/30.0))); // FT[Ya] factory trim calculation
  factoryTrim[2] = (mpu_accel_scaleFactor[SELFTEST_ACCEL_RANGE]*0.34)*(pow((0.92/0.34), (((float)selfTest[2] - 1.0)/30.0))); // FT[Za] factory trim calculation
  factoryTrim[3] = ( 25.0*mpu_gyro_scaleFactor[SELFTEST_GYRO_RANGE]/10.0)*(pow(1.046, ((float)selfTest[3] - 1.0) ));         // FT[Xg] factory trim calculation
  factoryTrim[4] = (-25.0*mpu_gyro_scaleFactor[SELFTEST_GYRO_RANGE]/10.0)*(pow(1.046, ((float)selfTest[4] - 1.0) ));         // FT[Yg] factory trim calculation
  factoryTrim[5] = ( 25.0*mpu_gyro_scaleFactor[SELFTEST_GYRO_RANGE]/10.0)*(pow(1.046, ((float)selfTest[5] - 1.0) ));         // FT[Zg] factory trim calculation

  // Report results as a ratio of (STR - FT)/FT; the change from Factory Trim of the Self-Test Response
  for (int i = 0; i < 6; i++) {
    result[i] = 100.0 + 100.0*((float)selfTest[i] - factoryTrim[i])/factoryTrim[i]; // Report percent differences
  }  
  if(result[0] < 1.0f && result[1] < 1.0f && result[2] < 1.0f && result[3] < 1.0f && result[4] < 1.0f && result[5] < 1.0f) {
    sprintf (buffer, "%s passed self-test (A_dev%%:[%.2f,%.2f,%.2f],G_dev%%:[%.2f,%.2f,%.2f])", mpu_sensor, result[0], result[1], result[2], result[3], result[4], result[5]);
    publish_event (STS_ESP32, SS_MOTION, EVENT_INIT, buffer);
    return true;  
  }
  else {
    sprintf (buffer, "%s failed self-test (A_dev%%:[%.2f,%.2f,%.2f],G_dev%%:[%.2f,%.2f,%.2f])", mpu_sensor, result[0], result[1], result[2], result[3], result[4], result[5]);
    publish_event (STS_ESP32, SS_MOTION, EVENT_WARNING, buffer);
    return false;    
  }
  #endif
}    

bool mpu_acquire() {
  static int16_t mpu_accel_raw_x, mpu_accel_raw_y, mpu_accel_raw_z;
  static int16_t mpu_gyro_raw_x, mpu_gyro_raw_y, mpu_gyro_raw_z;
  #ifdef MPU_9250
  static int16_t mpu_magn_raw_x, mpu_magn_raw_y, mpu_magn_raw_z;
  #endif
  static int16_t gravity_z;
  static int64_t mpu_accel_xy2; 

  if (mpu.testConnection()) {
    esp32.motion_active = true;
    radio.motion_active = true;
    motion.millis = millis();
    #ifdef MPU_6050
    mpu.getMotion6(&mpu_accel_raw_z, &mpu_accel_raw_x, &mpu_accel_raw_y, &mpu_gyro_raw_z, &mpu_gyro_raw_x, &mpu_gyro_raw_y);  // axes changed for rocket orientation: x_sensor=z_rocket, y_sensor=x_rocket, z_sensor=y_rocket 
    #endif
    #ifdef MPU_9250
    mpu.getMotion9(&mpu_accel_raw_z, &mpu_accel_raw_x, &mpu_accel_raw_y, &mpu_gyro_raw_z, &mpu_gyro_raw_x, &mpu_gyro_raw_y, &mpu_magn_raw_z, &mpu_magn_raw_x, &mpu_magn_raw_y);  // axes changed for rocket orientation: x_sensor=z_rocket, y_sensor=x_rocket, z_sensor=y_rocket 
    #endif
    
    if (config_esp32.motion_udp_raw_enable) {
      sprintf (buffer, "%8d,%8d,%8d,%8d,%8d,%8d,%8d,%8d,%8d,%8d,%8d,%8d", mpu_accel_raw_x, mpu_accel_raw_y, mpu_accel_raw_z, 
               mpu_accel_raw_x - config_esp32.mpu_accel_offset_x/mpu_ratioFactor[motion.accel_range], mpu_accel_raw_y - config_esp32.mpu_accel_offset_y/mpu_ratioFactor[motion.accel_range], mpu_accel_raw_z - config_esp32.mpu_accel_offset_z/mpu_ratioFactor[motion.accel_range],
               mpu_gyro_raw_x, mpu_gyro_raw_y, mpu_gyro_raw_z, 
               mpu_gyro_raw_x - config_esp32.mpu_gyro_offset_x/mpu_ratioFactor[motion.gyro_range], mpu_gyro_raw_y - config_esp32.mpu_gyro_offset_y/mpu_ratioFactor[motion.gyro_range], mpu_gyro_raw_z - config_esp32.mpu_gyro_offset_z/mpu_ratioFactor[motion.gyro_range]);
      publish_udp_text (buffer);
    }
    motion.accel_x = -(mpu_accel_raw_x - config_esp32.mpu_accel_offset_x/mpu_ratioFactor[motion.accel_range])*config_esp32.mpu_accel_sensitivity*mpu_ratioFactor[motion.accel_range]/10000; // [cm/s2]
    motion.accel_y = -(mpu_accel_raw_y - config_esp32.mpu_accel_offset_y/mpu_ratioFactor[motion.accel_range])*config_esp32.mpu_accel_sensitivity*mpu_ratioFactor[motion.accel_range]/10000; // [cm/s2]
    motion.accel_z = -(mpu_accel_raw_z - config_esp32.mpu_accel_offset_z/mpu_ratioFactor[motion.accel_range])*config_esp32.mpu_accel_sensitivity*mpu_ratioFactor[motion.accel_range]/10000; // [cm/s2]
    motion.gyro_x = 1000*(mpu_gyro_raw_x - config_esp32.mpu_gyro_offset_x)/mpu_gyro_scaleFactor[motion.gyro_range]; // [cdeg/s]
    motion.gyro_y = 1000*(mpu_gyro_raw_y - config_esp32.mpu_gyro_offset_y)/mpu_gyro_scaleFactor[motion.gyro_range]; // [cdeg/s]
    motion.gyro_z = 1000*(mpu_gyro_raw_z - config_esp32.mpu_gyro_offset_z)/mpu_gyro_scaleFactor[motion.gyro_range]; // [cdeg/s]
    #ifdef MPU_9250
    motion.magn_x = (int16_t) mpu_magn_raw_x * 1200 / 4096;
    motion.magn_y = (int16_t) mpu_magn_raw_y * 1200 / 4096;
    motion.magn_z = (int16_t) mpu_magn_raw_z * 1200 / 4096;
    #endif
    mpu_accel_xy2 = motion.accel_x*motion.accel_x + motion.accel_y*motion.accel_y;
    motion.g = uint16_t (1000*sqrt(float(mpu_accel_xy2 + motion.accel_z*motion.accel_z))/float(gravity_constant)); // [mG]
    if (esp32.state == STATE_THRUST) {
      gravity_z = - int16_t (sqrt (max((int64_t)0, gravity_constant*gravity_constant - mpu_accel_xy2))); // [cm/s2]      
    }
    else {
      gravity_z = sign (motion.accel_z) * int16_t (sqrt (max((int64_t)0, gravity_constant*gravity_constant - mpu_accel_xy2))); // [cm/s2]
    }
    motion.tilt = int16_t ((18000.0/M_PI)*acos(min(float(1.0), max(float(-1.0), -float(gravity_z)/float(gravity_constant))))); // /cdeg]
    motion.a = motion.accel_z - gravity_z; // [cm/s2]
    motion.rpm = motion.gyro_z/6; // [crpm]
    if (esp32.opsmode == MODE_CHECKOUT) {
      mpu_zero_gyro (mpu_gyro_raw_x, mpu_gyro_raw_y, mpu_gyro_raw_z);
      if (motion.g > 950 and motion.g < 1050) { // TODO: determine appropriate limits
        motion.accel_valid = true;  
      }
      else {
        motion.accel_valid = false;  
      }
    }
    else {
      if (motion.g < 100) {
        if (esp32.state != STATE_FREEFALL) { // TODO: determine appropriate limits
          publish_event (STS_ESP32, SS_MOTION, EVENT_INFO, "Start of free-fall detected");
          esp32.state = STATE_FREEFALL;
        }
      }
      else if (900 < motion.g and motion.g < 1100) { // TODO: determine appropriate limits
        if (esp32.state != STATE_STATIC) {
          publish_event (STS_ESP32, SS_MOTION, EVENT_INFO, "Static state detected");
          esp32.state = STATE_STATIC;
        }
      }
      else {
        if (esp32.separation_sts) {
          if (esp32.state != STATE_PARACHUTE) {
            publish_event (STS_ESP32, SS_MOTION, EVENT_INFO, "Start of parachute descent detected");
            esp32.state = STATE_PARACHUTE;
          }
        }
        else {
          if (esp32.state != STATE_THRUST) {
            publish_event (STS_ESP32, SS_MOTION, EVENT_INFO, "Start of thrust detected");
            esp32.state = STATE_THRUST;
          }
        }
      }
    }
    return true;
  }
  else {
    // lost connection, attempt reset  TODO: does this work and is this the way to proceed?
    publish_event (STS_ESP32, SS_MOTION, EVENT_WARNING, "Lost connection to MPU; attempting reset");
    return false;
  }
}

void mpu_zero_gyro (int16_t new_gyro_x, int16_t new_gyro_y, int16_t new_gyro_z) {
  static int16_t gyro_x_buffer[GYRO_BUFFER_SIZE];
  static int16_t gyro_y_buffer[GYRO_BUFFER_SIZE];
  static int16_t gyro_z_buffer[GYRO_BUFFER_SIZE];
  static int64_t gyro_x_sum, gyro_y_sum, gyro_z_sum;
  static uint8_t gyro_buffer_pos;
  static bool gyro_zero_level_ok;
  if (motion.gyro_valid and (abs(new_gyro_x - config_esp32.mpu_gyro_offset_x) > 20 or abs(new_gyro_y - config_esp32.mpu_gyro_offset_y) > 20 or abs(new_gyro_z - config_esp32.mpu_gyro_offset_z) > 20)) {
    // rocket moved too much for stable calibration, start again
    gyro_buffer_pos = 0;
    gyro_zero_level_ok = false;
    motion.gyro_valid = false;  
  }
  gyro_x_sum = gyro_x_sum - gyro_x_buffer[gyro_buffer_pos] + new_gyro_x;
  gyro_y_sum = gyro_y_sum - gyro_y_buffer[gyro_buffer_pos] + new_gyro_y;
  gyro_z_sum = gyro_z_sum - gyro_z_buffer[gyro_buffer_pos] + new_gyro_z;
  gyro_x_buffer[gyro_buffer_pos] = new_gyro_x;
  gyro_y_buffer[gyro_buffer_pos] = new_gyro_y;
  gyro_z_buffer[gyro_buffer_pos] = new_gyro_z;
  if (++gyro_buffer_pos == GYRO_BUFFER_SIZE) {
    gyro_buffer_pos = 0;
    gyro_zero_level_ok = true;
    motion.gyro_valid = true;
  }
  if (gyro_zero_level_ok) {
    config_esp32.mpu_gyro_offset_x = gyro_x_sum / GYRO_BUFFER_SIZE;
    config_esp32.mpu_gyro_offset_y = gyro_y_sum / GYRO_BUFFER_SIZE;
    config_esp32.mpu_gyro_offset_z = gyro_z_sum / GYRO_BUFFER_SIZE;
  }
}

#ifdef MPU_6050
bool mpu6050_checkConfig() {
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
  char mpu6050_config0_reference[2*(end_address0-start_address0+1)+1] = "8303813f41ae00000000000028";
  char mpu6050_config0[2*(end_address0-start_address0+1)+1];
  char mpu6050_config1_reference[2*(end_address1-start_address1+1)+1] = "0000000000000000e7060000000000000000000000000000000000000000000000000000000002000005";
  char mpu6050_config1[2*(end_address1-start_address1+1)+1];
  char mpu6050_config2_reference[2*(end_address2-start_address2+1)+1] = "000000";
  char mpu6050_config2[2*(end_address2-start_address2+1)+1];

  if (mpu6050_checkMemory (start_address0, end_address0, mpu6050_config0, mpu6050_config0_reference) &&
      mpu6050_checkMemory (start_address1, end_address1, mpu6050_config1, mpu6050_config1_reference) &&
      mpu6050_checkMemory (start_address2, end_address2, mpu6050_config2, mpu6050_config2_reference)) {
    publish_event (STS_ESP32, SS_MOTION, EVENT_INIT, "MPU6050 configuration checked");
    return true;
  }
  else {
    return false;
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
    publish_event (STS_ESP32, SS_MOTION, EVENT_WARNING, buffer);
    return false;
  }
  else {
    return true;
  }
}

void mpu6050_printConfig() {
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

#endif // MOTION
