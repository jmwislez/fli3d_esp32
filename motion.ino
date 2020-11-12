/*
 * Fli3d - Accelerometer/Gyroscope functionality
 */

#ifdef MOTION

#include "I2Cdev.h"
#include "MPU6050.h"
#include "Wire.h"

// Settings and global variables for MPU6050

#define SELFTEST_ACCEL_RANGE 0
#define SELFTEST_GYRO_RANGE  0
#define ACCEL_BUFFER_SIZE    50
#define GYRO_BUFFER_SIZE     50

const int16_t gravity_constant = 981; // [cm/s2]
const int16_t mpu6050_accel_scaleFactor[4] = { 16384, 8192, 4096, 2048 }; 
const int16_t mpu6050_gyro_scaleFactor[4] = { 1310, 655, 328, 164 }; // multiplied by 10 to allow for int calculations
const int8_t  mpu6050_ratioFactor[4] = { 1, 2, 4, 8 };

MPU6050 MPU6050;

bool motion_setup () {
  return (mpu6050_setup());
}

bool mpu6050_setup () {
  // Set up I2C bus
  Wire.begin (I2C_SDA_PIN, I2C_SCL_PIN); 
  Wire.setClock (400000);
  // Check if we find a properly responding MPU6050 device
  if (!mpu6050_testConnection ()) {
    return false;
  }
  // Do self-test
  mpu6050_selfTest (); 
  MPU6050.reset();
  delay(500);
  // Configure the MPU6050 for use
  MPU6050.setI2CBypassEnabled (true);
  delay (200);
  MPU6050.setI2CMasterModeEnabled(false);
  delay (200);
  MPU6050.setClockSource(MPU6050_CLOCK_PLL_XGYRO);
  delay (200);
  MPU6050.setSleepEnabled(false);
  delay (200);
  motion_set_samplerate (config_esp32.motion_rate);
  delay (200);
  MPU6050.setFullScaleGyroRange(mpu6050.gyro_range);
  delay (200);
  MPU6050.setFullScaleAccelRange(mpu6050.accel_range);
  delay (200);
  MPU6050.setXAccelOffset(0); // TODO: after fully understanding MPU6050, use this to set offset (likely allows use of full range)
  delay (200);
  MPU6050.setYAccelOffset(0); // TODO: after fully understanding MPU6050, use this to set offset (likely allows use of full range)
  delay (200);
  MPU6050.setZAccelOffset(0); // TODO: after fully understanding MPU6050, use this to set offset (likely allows use of full range)
  delay (200);
  publish_event (STS_ESP32, SS_MPU6050, EVENT_INIT, "Initialized MPU6050 accelerometer/gyroscope sensor");
  return true;
}

void motion_set_samplerate (uint8_t rate) { // TODO: rate is limited to 255, while sensor can do more
  uint8_t dlpf_bw;
  config_esp32.motion_rate = rate;
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
  MPU6050.setRate((uint8_t)((1000 - config_esp32.motion_rate) / config_esp32.motion_rate)); // TODO: is it meaningful to set the rate in case of polling?
  MPU6050.setDLPFMode(dlpf_bw);
}

void mpu6050_set_accel_range (uint8_t accel_range) {
  MPU6050.setFullScaleAccelRange(accel_range);  
  mpu6050.accel_range = MPU6050.getFullScaleAccelRange();
}

void mpu6050_set_gyro_range (uint8_t gyro_range) {
  MPU6050.setFullScaleGyroRange(gyro_range);
  mpu6050.gyro_range = MPU6050.getFullScaleGyroRange();
}

bool mpu6050_testConnection () {
  if (MPU6050.testConnection()) {
    publish_event (STS_ESP32, SS_MPU6050, EVENT_INIT, "Found MPU6050 6DOF motion sensor");
    return true;
  }
  else {
    publish_event (STS_ESP32, SS_MPU6050, EVENT_ERROR, "Failed to connect to MPU6050; disabling");
    return false;
  } 
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
  selfTest[0] = (rawData[0] >> 3) | (rawData[3] & 0x30) >> 4 ; // XA_TEST result is a five-bit charinteger
  selfTest[1] = (rawData[1] >> 3) | (rawData[3] & 0x0C) >> 2 ; // YA_TEST result is a five-bit charinteger
  selfTest[2] = (rawData[2] >> 3) | (rawData[3] & 0x03) >> 0 ; // ZA_TEST result is a five-bit charinteger
  // Extract the gyration test results first
  selfTest[3] = rawData[0]  & 0x1F ; // XG_TEST result is a five-bit charinteger
  selfTest[4] = rawData[1]  & 0x1F ; // YG_TEST result is a five-bit charinteger
  selfTest[5] = rawData[2]  & 0x1F ; // ZG_TEST result is a five-bit charinteger   
  // Process results to allow final comparison with factory set values
  factoryTrim[0] = (mpu6050_accel_scaleFactor[SELFTEST_ACCEL_RANGE]*0.34)*(pow((0.92/0.34), (((float)selfTest[0] - 1.0)/30.0))); // FT[Xa] factory trim calculation
  factoryTrim[1] = (mpu6050_accel_scaleFactor[SELFTEST_ACCEL_RANGE]*0.34)*(pow((0.92/0.34), (((float)selfTest[1] - 1.0)/30.0))); // FT[Ya] factory trim calculation
  factoryTrim[2] = (mpu6050_accel_scaleFactor[SELFTEST_ACCEL_RANGE]*0.34)*(pow((0.92/0.34), (((float)selfTest[2] - 1.0)/30.0))); // FT[Za] factory trim calculation
  factoryTrim[3] = ( 25.0*mpu6050_gyro_scaleFactor[SELFTEST_GYRO_RANGE]/10.0)*(pow(1.046, ((float)selfTest[3] - 1.0) ));         // FT[Xg] factory trim calculation
  factoryTrim[4] = (-25.0*mpu6050_gyro_scaleFactor[SELFTEST_GYRO_RANGE]/10.0)*(pow(1.046, ((float)selfTest[4] - 1.0) ));         // FT[Yg] factory trim calculation
  factoryTrim[5] = ( 25.0*mpu6050_gyro_scaleFactor[SELFTEST_GYRO_RANGE]/10.0)*(pow(1.046, ((float)selfTest[5] - 1.0) ));         // FT[Zg] factory trim calculation

  // Report results as a ratio of (STR - FT)/FT; the change from Factory Trim of the Self-Test Response
  for (int i = 0; i < 6; i++) {
    result[i] = 100.0 + 100.0*((float)selfTest[i] - factoryTrim[i])/factoryTrim[i]; // Report percent differences
  }  
  if(result[0] < 1.0f && result[1] < 1.0f && result[2] < 1.0f && result[3] < 1.0f && result[4] < 1.0f && result[5] < 1.0f) {
    sprintf (buffer, "MPU6050 passed self-test (A_dev%%:[%.2f,%.2f,%.2f],G_dev%%:[%.2f,%.2f,%.2f])", result[0], result[1], result[2], result[3], result[4], result[5]);
    publish_event (STS_ESP32, SS_MPU6050, EVENT_INIT, buffer);
    return true;  
  }
  else {
    sprintf (buffer, "MPU6050 failed self-test (A_dev%%:[%.2f,%.2f,%.2f],G_dev%%:[%.2f,%.2f,%.2f])", result[0], result[1], result[2], result[3], result[4], result[5]);
    publish_event (STS_ESP32, SS_MPU6050, EVENT_WARNING, buffer);
    return false;    
  }
}    

bool mpu6050_acquire () {
  static int16_t mpu6050_accel_raw_x, mpu6050_accel_raw_y, mpu6050_accel_raw_z;
  static int16_t mpu6050_gyro_raw_x, mpu6050_gyro_raw_y, mpu6050_gyro_raw_z;
  static int16_t gravity_z;
  static int64_t mpu6050_accel_xy2; 

  if (MPU6050.testConnection ()) {
    esp32.motion_active = true;
    radio.motion_active = true;
    mpu6050.millis = millis();
    MPU6050.getMotion6(&mpu6050_accel_raw_z, &mpu6050_accel_raw_x, &mpu6050_accel_raw_y, &mpu6050_gyro_raw_z, &mpu6050_gyro_raw_x, &mpu6050_gyro_raw_y);  // axes changed for rocket orientation: x_sensor=z_rocket, y_sensor=x_rocket, z_sensor=y_rocket 
    if (config_esp32.motion_udp_raw_enable) {
      sprintf (buffer, "%8d,%8d,%8d,%8d,%8d,%8d,%8d,%8d,%8d,%8d,%8d,%8d", mpu6050_accel_raw_x, mpu6050_accel_raw_y, mpu6050_accel_raw_z, 
               mpu6050_accel_raw_x - config_esp32.mpu6050_accel_offset_x/mpu6050_ratioFactor[mpu6050.accel_range], mpu6050_accel_raw_y - config_esp32.mpu6050_accel_offset_y/mpu6050_ratioFactor[mpu6050.accel_range], mpu6050_accel_raw_z - config_esp32.mpu6050_accel_offset_z/mpu6050_ratioFactor[mpu6050.accel_range],
               mpu6050_gyro_raw_x, mpu6050_gyro_raw_y, mpu6050_gyro_raw_z, 
               mpu6050_gyro_raw_x - config_esp32.mpu6050_gyro_offset_x/mpu6050_ratioFactor[mpu6050.gyro_range], mpu6050_gyro_raw_y - config_esp32.mpu6050_gyro_offset_y/mpu6050_ratioFactor[mpu6050.gyro_range], mpu6050_gyro_raw_z - config_esp32.mpu6050_gyro_offset_z/mpu6050_ratioFactor[mpu6050.gyro_range]);
      publish_udp_text (buffer);
    }
    mpu6050.accel_x = -(mpu6050_accel_raw_x - config_esp32.mpu6050_accel_offset_x/mpu6050_ratioFactor[mpu6050.accel_range])*config_esp32.mpu6050_accel_sensitivity*mpu6050_ratioFactor[mpu6050.accel_range]/10000; // [cm/s2]
    mpu6050.accel_y = -(mpu6050_accel_raw_y - config_esp32.mpu6050_accel_offset_y/mpu6050_ratioFactor[mpu6050.accel_range])*config_esp32.mpu6050_accel_sensitivity*mpu6050_ratioFactor[mpu6050.accel_range]/10000; // [cm/s2]
    mpu6050.accel_z = -(mpu6050_accel_raw_z - config_esp32.mpu6050_accel_offset_z/mpu6050_ratioFactor[mpu6050.accel_range])*config_esp32.mpu6050_accel_sensitivity*mpu6050_ratioFactor[mpu6050.accel_range]/10000; // [cm/s2]
    mpu6050.gyro_x = 1000*(mpu6050_gyro_raw_x - config_esp32.mpu6050_gyro_offset_x)/mpu6050_gyro_scaleFactor[mpu6050.gyro_range]; // [cdeg/s]
    mpu6050.gyro_y = 1000*(mpu6050_gyro_raw_y - config_esp32.mpu6050_gyro_offset_y)/mpu6050_gyro_scaleFactor[mpu6050.gyro_range]; // [cdeg/s]
    mpu6050.gyro_z = 1000*(mpu6050_gyro_raw_z - config_esp32.mpu6050_gyro_offset_z)/mpu6050_gyro_scaleFactor[mpu6050.gyro_range]; // [cdeg/s]
    mpu6050_accel_xy2 = mpu6050.accel_x*mpu6050.accel_x + mpu6050.accel_y*mpu6050.accel_y;
    mpu6050.g = uint16_t (1000*sqrt(float(mpu6050_accel_xy2 + mpu6050.accel_z*mpu6050.accel_z))/float(gravity_constant)); // [mG]
    if (esp32.state == STATE_THRUST) {
      gravity_z = - int16_t (sqrt (max((int64_t)0, gravity_constant*gravity_constant - mpu6050_accel_xy2))); // [cm/s2]      
    }
    else {
      gravity_z = sign (mpu6050.accel_z) * int16_t (sqrt (max((int64_t)0, gravity_constant*gravity_constant - mpu6050_accel_xy2))); // [cm/s2]
    }
    mpu6050.tilt = int16_t ((18000.0/M_PI)*acos(min(float(1.0), max(float(-1.0), -float(gravity_z)/float(gravity_constant))))); // /cdeg]
    mpu6050.a = mpu6050.accel_z - gravity_z; // [cm/s2]
    mpu6050.rpm = mpu6050.gyro_z/6; // [crpm]
    Serial.println (String("DEBUG: ") + mpu6050.accel_z + " " + gravity_z + " " + String(float(gravity_constant*gravity_constant - mpu6050_accel_xy2)) + " " + String(-float(gravity_z)/float(gravity_constant)));
    if (esp32.opsmode == MODE_CHECKOUT) {
      mpu6050_zero_gyro (mpu6050_gyro_raw_x, mpu6050_gyro_raw_y, mpu6050_gyro_raw_z);
      if (mpu6050.g > 950 and mpu6050.g < 1050) {
        mpu6050.accel_valid = true;  
      }
      else {
        mpu6050.accel_valid = false;  
      }
    }
    else {
      if (mpu6050.g < 100) {
        if (esp32.state != STATE_FREEFALL) { // TODO: determine appropriate limits
          publish_event (STS_ESP32, SS_MPU6050, EVENT_INFO, "Start of free-fall detected");
          esp32.state = STATE_FREEFALL;
        }
      }
      else if (950 < mpu6050.g and mpu6050.g < 1050) { // TODO: determine appropriate limits
        if (esp32.state != STATE_STATIC) {
          publish_event (STS_ESP32, SS_MPU6050, EVENT_INFO, "Static rocket detected");
          esp32.state = STATE_STATIC;
        }
      }
      else {
        if (esp32.separation_sts) {
          if (esp32.state != STATE_PARACHUTE) {
            publish_event (STS_ESP32, SS_MPU6050, EVENT_INFO, "Start of parachute descent detected");
            esp32.state = STATE_PARACHUTE;
          }
        }
        else {
          if (esp32.state != STATE_THRUST) {
            publish_event (STS_ESP32, SS_MPU6050, EVENT_INFO, "Start of thrust detected");
            esp32.state = STATE_THRUST;
          }
        }
      }
    }
    return true;
  }
  else {
    // lost connection, attempt reset  TODO: does this work and is this the way to proceed?
    publish_event (STS_ESP32, SS_MPU6050, EVENT_WARNING, "Lost connection to MPU6050; attempting reset");
    return false;
  }
}

void mpu6050_zero_gyro (int16_t new_gyro_x, int16_t new_gyro_y, int16_t new_gyro_z) {
  static int16_t gyro_x_buffer[GYRO_BUFFER_SIZE];
  static int16_t gyro_y_buffer[GYRO_BUFFER_SIZE];
  static int16_t gyro_z_buffer[GYRO_BUFFER_SIZE];
  static int64_t gyro_x_sum, gyro_y_sum, gyro_z_sum;
  static uint8_t gyro_buffer_pos;
  static bool gyro_zero_level_ok;
  if (mpu6050.gyro_valid and (abs(new_gyro_x - config_esp32.mpu6050_gyro_offset_x) > 20 or abs(new_gyro_y - config_esp32.mpu6050_gyro_offset_y) > 20 or abs(new_gyro_z - config_esp32.mpu6050_gyro_offset_z) > 20)) {
    // rocket moved too much for stable calibration, start again
    gyro_buffer_pos = 0;
    gyro_zero_level_ok = false;
    mpu6050.gyro_valid = false;  
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
    mpu6050.gyro_valid = true;
  }
  if (gyro_zero_level_ok) {
    config_esp32.mpu6050_gyro_offset_x = gyro_x_sum / GYRO_BUFFER_SIZE;
    config_esp32.mpu6050_gyro_offset_y = gyro_y_sum / GYRO_BUFFER_SIZE;
    config_esp32.mpu6050_gyro_offset_z = gyro_z_sum / GYRO_BUFFER_SIZE;
  }
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
  char mpu6050_config0_reference[2*(end_address0-start_address0+1)+1] = "8303813f41ae00000000000028";
  char mpu6050_config0[2*(end_address0-start_address0+1)+1];
  char mpu6050_config1_reference[2*(end_address1-start_address1+1)+1] = "0000000000000000e7060000000000000000000000000000000000000000000000000000000002000005";
  char mpu6050_config1[2*(end_address1-start_address1+1)+1];
  char mpu6050_config2_reference[2*(end_address2-start_address2+1)+1] = "000000";
  char mpu6050_config2[2*(end_address2-start_address2+1)+1];

  if (mpu6050_checkMemory (start_address0, end_address0, mpu6050_config0, mpu6050_config0_reference) &&
      mpu6050_checkMemory (start_address1, end_address1, mpu6050_config1, mpu6050_config1_reference) &&
      mpu6050_checkMemory (start_address2, end_address2, mpu6050_config2, mpu6050_config2_reference)) {
    publish_event (STS_ESP32, SS_MPU6050, EVENT_INIT, "MPU6050 configuration checked");
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
  Serial.print (1000/(1+MPU6050.getRate()));
  Serial.println (" Hz");
  Serial.print ("DLFPMode:                     ");
  switch (MPU6050.getDLPFMode()) {
    case 0x00: Serial.println ("MPU6050_DLPF_BW_256"); break;
    case 0x01: Serial.println ("MPU6050_DLPF_BW_188"); break;
    case 0x02: Serial.println ("MPU6050_DLPF_BW_98"); break;
    case 0x03: Serial.println ("MPU6050_DLPF_BW_42"); break;
    case 0x04: Serial.println ("MPU6050_DLPF_BW_20"); break;
    case 0x05: Serial.println ("MPU6050_DLPF_BW_10"); break;
    case 0x06: Serial.println ("MPU6050_DLPF_BW_5"); break;
  }
  Serial.print ("FullScaleGyroRange:           +/- ");
  Serial.print (FullScaleGyroRange[MPU6050.getFullScaleGyroRange()]);
  Serial.println (" deg/s");
  Serial.print ("FullScaleAccelRange:          +/- ");
  Serial.print (FullScaleAccelRange[MPU6050.getFullScaleAccelRange()]);
  Serial.println (" g");
  Serial.print ("DHPFMode:                     ");
  switch (MPU6050.getDHPFMode()) {
    case 0x00: Serial.println ("MPU6050_DHPF_RESET"); break;
    case 0x01: Serial.println ("MPU6050_DHPF_5"); break;
    case 0x02: Serial.println ("MPU6050_DHPF_2P5"); break;
    case 0x03: Serial.println ("MPU6050_DHPF_1P25"); break;
    case 0x04: Serial.println ("MPU6050_DHPF_0P63"); break;
    case 0x07: Serial.println ("MPU6050_DHPF_HOLD"); break;
  }
  Serial.print ("FreefallDetectionThreshold:   ");
  Serial.println (MPU6050.getFreefallDetectionThreshold());
  Serial.print ("FreefallDetectionDuration:    ");
  Serial.println (MPU6050.getFreefallDetectionDuration());
  Serial.print ("MotionDetectionThreshold:     ");
  Serial.println (MPU6050.getMotionDetectionThreshold());
  Serial.print ("MotionDetectionDuration:      ");
  Serial.println (MPU6050.getMotionDetectionDuration());
  Serial.print ("ZeroMotionDetectionThreshold: ");
  Serial.println (MPU6050.getZeroMotionDetectionThreshold());
  Serial.print ("ZeroMotionDetectionDuration:  ");
  Serial.println (MPU6050.getZeroMotionDetectionDuration());
  Serial.print ("TempFIFOEnabled:              ");
  Serial.println (MPU6050.getTempFIFOEnabled());
  Serial.print ("XGyroFIFOEnabled:             ");
  Serial.println (MPU6050.getXGyroFIFOEnabled());
  Serial.print ("YGyroFIFOEnabled:             ");
  Serial.println (MPU6050.getYGyroFIFOEnabled());
  Serial.print ("ZGyroFIFOEnabled:             ");
  Serial.println (MPU6050.getZGyroFIFOEnabled());
  Serial.print ("AccelFIFOEnabled:             ");
  Serial.println (MPU6050.getAccelFIFOEnabled());
  Serial.print ("MasterClockSpeed:             ");
  Serial.println (MPU6050.getMasterClockSpeed());
  Serial.print ("PassthroughStatus:            ");
  Serial.println (MPU6050.getPassthroughStatus());
  Serial.print ("InterruptMode:                ");
  Serial.println (MPU6050.getInterruptMode());
  Serial.print ("InterruptDrive:               ");
  Serial.println (MPU6050.getInterruptDrive());
  Serial.print ("InterruptLatch:               ");
  Serial.println (MPU6050.getInterruptLatch());
  Serial.print ("InterruptLatchClear:          ");
  Serial.println (MPU6050.getInterruptLatchClear());
  Serial.print ("I2CBypassEnabled:             ");
  Serial.println (MPU6050.getI2CBypassEnabled());
  Serial.print ("ClockOutputEnabled:           ");
  Serial.println (MPU6050.getClockOutputEnabled());
  Serial.print ("IntEnabled:                   ");
  Serial.println (MPU6050.getIntEnabled());
  Serial.print ("IntFreefallEnabled:           ");
  Serial.println (MPU6050.getIntFreefallEnabled());
  Serial.print ("IntMotionEnabled:             ");
  Serial.println (MPU6050.getIntMotionEnabled());
  Serial.print ("IntZeroMotionEnabled:         ");
  Serial.println (MPU6050.getIntZeroMotionEnabled());
  Serial.print ("IntFIFOBufferOverflowEnabled: ");
  Serial.println (MPU6050.getIntFIFOBufferOverflowEnabled());
  Serial.print ("IntI2CMasterEnabled:          ");
  Serial.println (MPU6050.getIntI2CMasterEnabled());
  Serial.print ("IntDataReadyEnabled:          ");
  Serial.println (MPU6050.getIntDataReadyEnabled());
  Serial.print ("FIFOEnabled:                  ");
  Serial.println (MPU6050.getFIFOEnabled());
  Serial.print ("I2CMasterModeEnabled:         ");
  Serial.println (MPU6050.getI2CMasterModeEnabled());
  Serial.print ("SleepEnabled:                 ");
  Serial.println (MPU6050.getSleepEnabled());
  Serial.print ("WakeCycleEnabled:             ");
  Serial.println (MPU6050.getWakeCycleEnabled());
  Serial.print ("TempSensorEnabled:            ");
  Serial.println (MPU6050.getTempSensorEnabled());
  Serial.print ("ClockSource:                  ");
  Serial.println (MPU6050.getClockSource());
  Serial.print ("DeviceID:                     ");
  Serial.println (MPU6050.getDeviceID());
  Serial.print ("OTPBankValid:                 ");
  Serial.println (MPU6050.getOTPBankValid());
  Serial.print ("X_gyro_offsetTC:              ");
  Serial.println (MPU6050.getXGyroOffsetTC());
  Serial.print ("Y_gyro_offsetTC:              ");
  Serial.println (MPU6050.getYGyroOffsetTC());
  Serial.print ("Z_gyro_offsetTC:              ");
  Serial.println (MPU6050.getZGyroOffsetTC());
  Serial.print ("XFineGain:                    ");
  Serial.println (MPU6050.getXFineGain());
  Serial.print ("YFineGain:                    ");
  Serial.println (MPU6050.getYFineGain());
  Serial.print ("ZFineGain:                    ");
  Serial.println (MPU6050.getZFineGain());
  Serial.print ("X_accel_offset:               ");
  Serial.println (MPU6050.getXAccelOffset());
  Serial.print ("Y_accel_offset:               ");
  Serial.println (MPU6050.getYAccelOffset());
  Serial.print ("Z_accel_offset:               ");
  Serial.println (MPU6050.getZAccelOffset());
  Serial.print ("X_gyro_offset:                ");
  Serial.println (MPU6050.getXGyroOffset());
  Serial.print ("Y_gyro_offset:                ");
  Serial.println (MPU6050.getYGyroOffset());
  Serial.print ("Z_gyro_offset:                ");
  Serial.println (MPU6050.getZGyroOffset());
  Serial.print ("IntPLLReadyEnabled:           ");
  Serial.println (MPU6050.getIntPLLReadyEnabled());
  Serial.print ("IntDMPEnabled:                ");
  Serial.println (MPU6050.getIntDMPEnabled());
  Serial.print ("DMPEnabled:                   ");
  Serial.println (MPU6050.getDMPEnabled());
  Serial.print ("DMPConfig1:                   ");
  Serial.println (MPU6050.getDMPConfig1());
  Serial.print ("DMPConfig2:                   ");
  Serial.println (MPU6050.getDMPConfig2());
  Serial.println ("============================================================");
}

#endif // MOTION
