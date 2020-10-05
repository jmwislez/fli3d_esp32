/*
 * Fli3d - pressure sensor functionality
 */

#ifdef PRESSURE

#include <Wire.h>
#include <SPI.h>
#include "I2Cdev.h"

#define PRESSURE_BUFFER_SIZE 50

#define BMP280_I2C_Address           0x76 // Nominal address for BMP280 is 0x77
#define BMP280_DEVICE_ID             0x58 // The BMP280 device id
#define BMP280_RESET_CODE            0xB6 // The BMP280 reset code

#define BMP280_REGISTER_TRIM_PARAMS  0x88 // Trim parameter registers' base sub-address
#define BMP280_REGISTER_DEVICE_ID    0xD0 // Device ID register sub-address
#define BMP280_REGISTER_RESET        0xE0 // Reset register sub-address
#define BMP280_REGISTER_STATUS       0xF3 // Status register sub-address
#define BMP280_REGISTER_CTRL_MEAS    0xF4 // Control and measurement register sub-address
#define BMP280_REGISTER_CONFIG       0xF5 // Configuration register sub-address
#define BMP280_REGISTER_PRES_MSB     0xF7 // Pressure Most Significant Byte (MSB) register sub-address
#define BMP280_REGISTER_PRES_LSB     0xF8 // Pressure Least Significant Byte (LSB) register sub-address
#define BMP280_REGISTER_PRES_XLSB    0xF9 // Pressure eXtended Least Significant Byte (XLSB) register sub-address
#define BMP280_REGISTER_TEMP_MSB     0xFA // Pressure Most Significant Byte (MSB) register sub-address
#define BMP280_REGISTER_TEMP_LSB     0xFB // Pressure Least Significant Byte (LSB) register sub-address
#define BMP280_REGISTER_TEMP_XLSB    0xFC // Pressure eXtended Least Significant Byte (XLSB) register sub-address

// BMP280 Device mode bitfield in the control and measurement register
#define BMP280_MODE_BIT                 1
#define BMP280_MODE_LENGTH              2
#define SLEEP_MODE                   0x00           
#define FORCED_MODE                  0x01
#define NORMAL_MODE                  0x03
// BMP280 Oversampling bit fields in the control and measurement register
#define BMP280_OSRS_T_BIT               7
#define BMP280_OSRS_T_LENGTH            3
#define BMP280_OSRS_P_BIT               4
#define BMP280_OSRS_P_LENGTH            3
#define OVERSAMPLING_SKIP            0x00          
#define OVERSAMPLING_X1              0x01
#define OVERSAMPLING_X2              0x02
#define OVERSAMPLING_X4              0x03
#define OVERSAMPLING_X8              0x04
#define OVERSAMPLING_X16             0x05
// BMP280 Infinite Impulse Response (IIR) filter bit field in the configuration register
#define BMP280_FILTER_BIT               4
#define BMP280_FILTER_LENGTH            3
#define IIR_FILTER_OFF               0x00          
#define IIR_FILTER_2                 0x01
#define IIR_FILTER_4                 0x02
#define IIR_FILTER_8                 0x03
#define IIR_FILTER_16                0x04
// BMP280 Time standby bit field in the configuration register
#define BMP280_T_SB_BIT                 7
#define BMP280_T_SB_LENGTH              3
#define TIME_STANDBY_05MS            0x00          
#define TIME_STANDBY_62MS            0x01
#define TIME_STANDBY_125MS           0x02
#define TIME_STANDBY_250MS           0x03
#define TIME_STANDBY_500MS           0x04
#define TIME_STANDBY_1000MS          0x05
#define TIME_STANDBY_2000MS          0x06
#define TIME_STANDBY_4000MS          0x07
// BMP280 status register bit fields
#define BMP280_UPDATE_BIT               0
#define BMP280_MEASURING_BIT            3


// Settings and global variables for BMP280
float zero_level_pressure = 1013.0f;
float bmp280_old_altitude;
uint32_t bmp280_millis, bmp280_old_millis;

struct {                                  
  // The BMP280 compensation trim parameters (coefficients)
  uint16_t dig_T1;
  int16_t  dig_T2;
  int16_t  dig_T3;
  uint16_t dig_P1;
  int16_t  dig_P2;
  int16_t  dig_P3;
  int16_t  dig_P4;
  int16_t  dig_P5;
  int16_t  dig_P6;
  int16_t  dig_P7;
  int16_t  dig_P8;
  int16_t  dig_P9;
} params;

int32_t t_fine;


bool pressure_setup () {
  return (bmp280_setup ());
}

bool bmp280_setup () {  
  
  if (!esp32.motion_enabled) {
    bus_publish_event (STS_ESP32, SS_BMP280, EVENT_WARNING, "MPU6050 not initialized; configuring MPU6050 I2C pass-through for BMP280");
    Wire.begin (I2C_SDA_PIN, I2C_SCL_PIN); 
    Wire.setClock (400000);
    //mpu.setI2CBypassEnabled (true);
    I2Cdev::writeBit(0x68, 0x37, 1, true);
    //mpu.setI2CMasterModeEnabled(false);
    I2Cdev::writeBit(0x68, 0x6A, 5, false);
  }

  // Reset BMP280
  bmp280_reset ();

  // Check if we find a properly responding BMP280 device
  if (bmp280_getDeviceId () == (uint8_t)BMP280_DEVICE_ID) {
    bus_publish_event (STS_ESP32, SS_BMP280, EVENT_INIT, "Found BMP280 barometric pressure sensor");
  }
  else {
    bus_publish_event (STS_ESP32, SS_BMP280, EVENT_ERROR, "Failed to connect to BMP280; disabling");
    return false;
  }

  // Load the calibration parameters from the sensor
  bmp280_getTrimParameters ();

  // Configure BMP280
  uint8_t timeStandby = TIME_STANDBY_05MS; // 000
  uint8_t iirFilter = IIR_FILTER_OFF;      // 000
  uint8_t mode = NORMAL_MODE;              // 11
  uint8_t tempOversampling;
  uint8_t presOversampling; 
  if (config_esp32.pressure_rate <= 23.1) {
    tempOversampling = OVERSAMPLING_X2;    // 010
    presOversampling = OVERSAMPLING_X16;   // 101
  } else if (config_esp32.pressure_rate <= 44.4) {
    tempOversampling = OVERSAMPLING_X1;    // 001
    presOversampling = OVERSAMPLING_X8;    // 100
  } else if (config_esp32.pressure_rate <= 75.0) {
    tempOversampling = OVERSAMPLING_X1;    // 001     
    presOversampling = OVERSAMPLING_X4;    // 011
  } else if (config_esp32.pressure_rate <= 114.6) {
    tempOversampling = OVERSAMPLING_X1;    // 001  
    presOversampling = OVERSAMPLING_X2;    // 010
  } else {
    tempOversampling = OVERSAMPLING_X1;    // 001  
    presOversampling = OVERSAMPLING_X1;    // 001
  }
  bmp280_setTimeStandby (timeStandby);
  bmp280_setFilter (iirFilter);
  bmp280_setTempOversampling (tempOversampling);
  bmp280_setPresOversampling (presOversampling);
  bmp280_setMode (mode);
  bus_publish_event (STS_ESP32, SS_BMP280, EVENT_INIT, "Initialized BMP280 barometric pressure sensor");
  return true;
}

bool bmp280_check () { 
  return !bmp280_getMeasuringStatus ();
}

bool bmp280_acquire () {
  static uint8_t bmp280_data[6];
  bmp280_old_altitude = bmp280.altitude;
  bmp280_old_millis = bmp280.millis;
  while (!bmp280_check()) { }
  if (bmp280_getDeviceId () == (uint8_t)BMP280_DEVICE_ID) {
    bmp280.millis = millis();
    I2Cdev::readBytes(BMP280_I2C_Address, BMP280_REGISTER_PRES_MSB, 6, &bmp280_data[0], 0);
    int32_t adcPres = (int32_t)bmp280_data[0] << 12 | (int32_t)bmp280_data[1] << 4 | (int32_t)bmp280_data[2] >> 4;
    uint32_t pres = bmp280_compensate_P_int64(adcPres);
    bmp280.pressure = (float)pres / 256.0f / 100.0f; // [Pa]
    int32_t adcTemp = (int32_t)bmp280_data[3] << 12 | (int32_t)bmp280_data[4] << 4 | (int32_t)bmp280_data[5] >> 4;
    int32_t temp = bmp280_compensate_T_int32(adcTemp);
    bmp280.temperature = (float)temp / 100.0f; // [degC]
    bmp280.altitude = ((float)powf(bmp280.zero_level_pressure / bmp280.pressure, 0.190223f) - 1.0f) * (bmp280.temperature + 273.15f) / 0.0065f; // [m]
    bmp280.velocity_v = 1000*(bmp280.altitude - bmp280_old_altitude) / (bmp280.millis - bmp280_old_millis); // [m/s] 
    if (esp32.opsmode == MODE_CHECKOUT) {
      bmp280.zero_level_pressure = bmp280_zero_level (bmp280.pressure);
    }
    return true;
  }
  else {
    // lost connection, attempt reset
    bus_publish_event (STS_ESP32, SS_BMP280, EVENT_WARNING, "Lost connection to BMP280; attempting reset");
    return false;
  }
}

float bmp280_zero_level (float new_pressure) {
  static float pressure_buffer[PRESSURE_BUFFER_SIZE];
  static float pressure_sum;
  static uint8_t pressure_buffer_pos;
  static bool zero_level_ok;
  pressure_sum = pressure_sum - pressure_buffer[pressure_buffer_pos] + new_pressure;
  pressure_buffer[pressure_buffer_pos] = new_pressure;
  if (++pressure_buffer_pos == PRESSURE_BUFFER_SIZE) {
    pressure_buffer_pos = 0;
    zero_level_ok = true;
  }
  if (zero_level_ok) {
    return (pressure_sum / PRESSURE_BUFFER_SIZE);
  }
  else {
    return (1013.0);
  }
}

bool bmp280_checkConfig () {
  uint8_t data;
  #define config1_len 2
  uint8_t config1_start = 0xF4;
  
  char bmp280_config1_reference[2*config1_len+1] = "5700";
  char bmp280_config1[2*config1_len+1];
  
  // read from 0xF4 to 0xF5 (2 bytes)
  for (uint8_t ctr=0; ctr<config1_len; ctr++) {
    I2Cdev::readByte(BMP280_I2C_Address, config1_start++, &data, 0);
    sprintf (&bmp280_config1[2*ctr], "%02x", data);
  }
  if (strcmp(bmp280_config1, bmp280_config1_reference)) {
    sprintf (buffer, "BMP280 configuration not as expected (%s)", bmp280_config1);
    bus_publish_event (STS_ESP32, SS_BMP280, EVENT_WARNING, buffer);
  }
  else {
    bus_publish_event (STS_ESP32, SS_BMP280, EVENT_INIT, "BMP280 configuration checked");
  }
}

void bmp280_printConfig () {
  Serial.println ("===== BMP280 status ========================================");
  Serial.print ("Mode:             ");
  switch (bmp280_getMode ()) {
    case 0: Serial.println ("SLEEP_MODE"); break;
    case 1: Serial.println ("FORCED_MODE"); break;
    case 3: Serial.println ("NORMAL_MODE"); break;
  }
  Serial.print ("iirFilter:        ");
  switch (bmp280_getFilter ()) {
    case 0: Serial.println ("IIR_FILTER_OFF"); break;
    case 1: Serial.println ("IIR_FILTER_2"); break;
    case 2: Serial.println ("IIR_FILTER_4"); break;
    case 3: Serial.println ("IIR_FILTER_8"); break;
    case 4: Serial.println ("IIR_FILTER_16"); break;
  }
  Serial.print ("tempOversampling: ");
  switch (bmp280_getTempOversampling ()) {
    case 0: Serial.println ("OVERSAMPLING_SKIP"); break;
    case 1: Serial.println ("OVERSAMPLING_X1"); break;
    case 2: Serial.println ("OVERSAMPLING_X2"); break;
    case 3: Serial.println ("OVERSAMPLING_X4"); break;
    case 4: Serial.println ("OVERSAMPLING_X8"); break;
    case 5: Serial.println ("OVERSAMPLING_X16"); break;
  }
  Serial.print ("presOversampling: ");
  switch (bmp280_getPresOversampling ()) {
    case 0: Serial.println ("OVERSAMPLING_SKIP"); break;
    case 1: Serial.println ("OVERSAMPLING_X1"); break;
    case 2: Serial.println ("OVERSAMPLING_X2"); break;
    case 3: Serial.println ("OVERSAMPLING_X4"); break;
    case 4: Serial.println ("OVERSAMPLING_X8"); break;
    case 5: Serial.println ("OVERSAMPLING_X16"); break;
  }
  Serial.print ("timeStandby:      ");
  switch (bmp280_getTimeStandby ()) {
    case 0: Serial.println ("TIME_STANDBY_05MS"); break;
    case 1: Serial.println ("TIME_STANDBY_62MS"); break;
    case 2: Serial.println ("TIME_STANDBY_125MS"); break;
    case 3: Serial.println ("TIME_STANDBY_250MS"); break;
    case 4: Serial.println ("TIME_STANDBY_500MS"); break;
    case 5: Serial.println ("TIME_STANDBY_1000MS"); break;
    case 6: Serial.println ("TIME_STANDBY_2000MS"); break;
    case 7: Serial.println ("TIME_STANDBY_4000MS"); break;
  }
  Serial.println ("============================================================");
}

// Support functions

int32_t bmp280_compensate_T_int32(int32_t adc_T) {
  // Returns temperature in DegC, resolution is 0.01 DegC. Output value of 5123 equals 51.23 DegC.
  // t_fine carries fine temperature as global value
  int32_t var1, var2, T;
  var1 = ((((adc_T >> 3) - ((int32_t)params.dig_T1 << 1))) * ((int32_t)params.dig_T2)) >> 11;
  var2 = (((((adc_T >> 4) - ((int32_t)params.dig_T1)) * ((adc_T >> 4) - ((int32_t)params.dig_T1))) >> 12) *
  ((int32_t)params.dig_T3)) >> 14;
  t_fine = var1 + var2;
  T = (t_fine * 5 + 128) >> 8;
  return T;
}

uint32_t bmp280_compensate_P_int64(int32_t adc_P) {
  // Returns pressure in Pa as unsigned 32 bit integer in Q24.8 format (24 integer bits and 8 fractional bits).
  // Output value of 24674867 represents 24674867/256 = 96386.2 Pa = 963.862 hPa
  int64_t var1, var2, p;
  var1 = ((int64_t)t_fine) - 128000;
  var2 = var1 * var1 * (int64_t)params.dig_P6;
  var2 = var2 + ((var1 * (int64_t)params.dig_P5) << 17);
  var2 = var2 + (((int64_t)params.dig_P4) << 35);
  var1 = ((var1 * var1 * (int64_t)params.dig_P3) >> 8) + ((var1 * (int64_t)params.dig_P2) << 12);
  var1 = (((((int64_t)1) << 47) + var1)) * ((int64_t)params.dig_P1) >> 33;
  if (var1 == 0){
    return 0; // avoid exception caused by division by zero
  }
  p = 1048576 - adc_P;
  p = (((p << 31) - var2) * 3125) / var1;
  var1 = (((int64_t)params.dig_P9) * (p >> 13) * (p>>13)) >> 25;
  var2 = (((int64_t)params.dig_P8) * p) >> 19;
  p = ((p + var1 + var2) >> 8) + (((int64_t)params.dig_P7) << 4);
  return (uint32_t)p;
}

// I/O functions for BMP280

uint8_t bmp280_getTimeStandby () {
  uint8_t bmp280_t_sb;
  I2Cdev::readBits(BMP280_I2C_Address, BMP280_REGISTER_CONFIG, BMP280_T_SB_BIT, BMP280_T_SB_LENGTH, &bmp280_t_sb, 0);
  return (bmp280_t_sb);
}

void bmp280_setTimeStandby (uint8_t bmp280_t_sb) {
  I2Cdev::writeBits(BMP280_I2C_Address, BMP280_REGISTER_CONFIG, BMP280_T_SB_BIT, BMP280_T_SB_LENGTH, bmp280_t_sb);
}

uint8_t bmp280_getFilter () {
  uint8_t bmp280_filter;
  I2Cdev::readBits(BMP280_I2C_Address, BMP280_REGISTER_CONFIG, BMP280_FILTER_BIT, BMP280_FILTER_LENGTH, &bmp280_filter, 0);
  return (bmp280_filter);
}

void bmp280_setFilter (uint8_t bmp280_filter) {
  I2Cdev::writeBits(BMP280_I2C_Address, BMP280_REGISTER_CONFIG, BMP280_FILTER_BIT, BMP280_FILTER_LENGTH, bmp280_filter);
}

uint8_t bmp280_getMode () {
  uint8_t bmp280_mode;
  I2Cdev::readBits(BMP280_I2C_Address, BMP280_REGISTER_CTRL_MEAS, BMP280_MODE_BIT, BMP280_MODE_LENGTH, &bmp280_mode, 0);
  return (bmp280_mode);
}

void bmp280_setMode (uint8_t bmp280_mode) {
  I2Cdev::writeBits(BMP280_I2C_Address, BMP280_REGISTER_CTRL_MEAS, BMP280_MODE_BIT, BMP280_MODE_LENGTH, bmp280_mode);
}

uint8_t bmp280_getPresOversampling () {
  uint8_t bmp280_osrs_p;
  I2Cdev::readBits(BMP280_I2C_Address, BMP280_REGISTER_CTRL_MEAS, BMP280_OSRS_P_BIT, BMP280_OSRS_P_LENGTH, &bmp280_osrs_p, 0);
  return (bmp280_osrs_p);
}

void bmp280_setPresOversampling (uint8_t bmp280_osrs_p) {
  I2Cdev::writeBits(BMP280_I2C_Address, BMP280_REGISTER_CTRL_MEAS, BMP280_OSRS_P_BIT, BMP280_OSRS_P_LENGTH, bmp280_osrs_p);
}

uint8_t bmp280_getTempOversampling () {
  uint8_t bmp280_osrs_t;
  I2Cdev::readBits(BMP280_I2C_Address, BMP280_REGISTER_CTRL_MEAS, BMP280_OSRS_T_BIT, BMP280_OSRS_T_LENGTH, &bmp280_osrs_t, 0);
  return (bmp280_osrs_t);
}

void bmp280_setTempOversampling (uint8_t bmp280_osrs_t) {
  I2Cdev::writeBits(BMP280_I2C_Address, BMP280_REGISTER_CTRL_MEAS, BMP280_OSRS_T_BIT, BMP280_OSRS_T_LENGTH, bmp280_osrs_t);
}

void bmp280_reset () {
  I2Cdev::writeByte(BMP280_I2C_Address, BMP280_REGISTER_RESET, BMP280_RESET_CODE);
  while (bmp280_getUpdateStatus ()) { }
}

uint8_t bmp280_getDeviceId () {
  uint8_t bmp280_device_id = 0;
  I2Cdev::readByte(BMP280_I2C_Address, BMP280_REGISTER_DEVICE_ID, &bmp280_device_id, 0);
  return (bmp280_device_id);
}

void bmp280_getTrimParameters () {
  I2Cdev::readBytes (BMP280_I2C_Address, BMP280_REGISTER_TRIM_PARAMS, sizeof(params), (uint8_t*)&params, 0);  
}

bool bmp280_getUpdateStatus () {
  static bool update = false;
  I2Cdev::readBit(BMP280_I2C_Address, BMP280_REGISTER_STATUS, BMP280_UPDATE_BIT, (uint8_t*)&update, 0);
  return (update);
}

bool bmp280_getMeasuringStatus () {
  static bool measuring = false;
  I2Cdev::readBit(BMP280_I2C_Address, BMP280_REGISTER_STATUS, BMP280_MEASURING_BIT, (uint8_t*)&measuring, 0);
  return (measuring);
}

#endif
