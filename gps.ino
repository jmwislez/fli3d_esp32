/*
 * Fli3d - gps functionality
 * 
 * configure NMEAGPS as follows:
 * - in NeoGPS/src/NMEAGPS_cfg.h, indicate the GPS sentences to be parsed: VTG (velocities), GGA (default), GST (cs + lat/lon/alt err), GSA (hdop/vdop/pdop), LAST_SENTENCE_IN_INTERVAL NMEAGPS::NMEA_GST (send order is VTG,GGA,GSA,GST)
 * - in NeoGPS/src/GPSfix_cfg.h, enable only GPS_FIX_TIME, GPS_FIX_LOCATION, GPS_FIX_ALTITUDE, GPS_FIX_SPEED, GPS_FIX_VELNED, GPS_FIX_HEADING, GPS_FIX_SATELLITES, GPS_FIX_HDOP, GPS_FIX_VDOP, GPS_FIX_PDOP, GPS_FIX_LAT_ERR, GPS_FIX_LON_ERR, GPS_FIX_ALT_ERR 
 */
 
#ifdef GPS

#include <NMEAGPS.h> 
#include <ublox/ubxGPS.h>

#define GPSBaud 57600 // tried 115200 but got garbage in some cases
#define SerialGPS Serial1
#define GPS_BUFFER_SIZE 50

const int32_t latitude_deg_to_m = M_PI*12742000/360;
const int32_t longitude_deg_to_m = M_PI*12742000*cos(51*M_PI/180)/360;

static NMEAGPS gps;
static gps_fix fix;

bool gps_setup() {
  uint32_t baud_rate;

  if ((baud_rate = find_gps_baudrate())) {
    sprintf (buffer, "Connected to GPS at %d baud", baud_rate);
    publish_event (STS_ESP32, SS_GPS, EVENT_INIT, buffer);
    gps_configure();
    publish_event (STS_ESP32, SS_GPS, EVENT_INIT, "Reconfiguring GPS");     
    if (baud_rate != GPSBaud) { 
      set_gps_baudrate (); 
      sprintf (buffer, "Set GPS to %d baud", GPSBaud);
      publish_event (STS_ESP32, SS_GPS, EVENT_INIT, buffer);  
    }
    gps_restart();
    if (check_gps_communication(baud_rate)) {
      sprintf (buffer, "Connected to GPS at %d baud", baud_rate);
      publish_event (STS_ESP32, SS_GPS, EVENT_INIT, buffer);
      return true;
    }
    else {
      publish_event (STS_ESP32, SS_GPS, EVENT_ERROR, "Failed to reconnect to GPS, disabling");
      return false;      
    }
  }
  else {
    publish_event (STS_ESP32, SS_GPS, EVENT_ERROR, "Failed to connect to GPS, disabling");
    return false;
  }
}

uint32_t find_gps_baudrate() {
  uint32_t gps_baudrate[6] = { 57600, 9600, 4800, 19200, 38400, 115200 };
  for (uint8_t i=0; i<6; i++) {
    delay (1000);
    SerialGPS.setPins(GPS_RX_PIN, GPS_TX_PIN);
    //SerialGPS.begin(gps_baudrate[i], SERIAL_8N1);
    SerialGPS.begin(gps_baudrate[i]);
    SerialGPS.flush();
    if (check_gps_communication (gps_baudrate[i])) {
      return gps_baudrate[i];
    }
  }
  return 0;
}

bool check_gps_communication (uint32_t baudrate) {
  char gps_char;
  uint8_t gps_good = 0;
  uint8_t gps_bad = 0;
  uint16_t ctr = 0;
  uint32_t start_probe_millis, now_millis;

  start_probe_millis = millis();
  now_millis = millis();
  while (now_millis - start_probe_millis < 5000) {
    while (SerialGPS.available() and gps_good < 255 and gps_bad < 255) {
      gps_char = SerialGPS.read();
      if (ctr++ > 200) {
        if ((gps_char >= ' ' and gps_char <= 'Z') or (uint8_t)gps_char == 10 or (uint8_t)gps_char == 13) {
          Serial.print(gps_char);
          gps_good++;
        }
        else {
          Serial.print("[");
          Serial.print(gps_char);
          Serial.print("]");
          gps_bad++;
        }
      }
    }
    now_millis = millis();
  } // timeout
  Serial.println();
  Serial.print("GPS ratio ");
  Serial.print(gps_good);
  Serial.print(":");
  Serial.println(gps_bad);   
  if (gps_good == 255) {
    // proper data flow received
    return true;
  }
  else if (gps_good == 0 and gps_bad == 0) {
    // timeout
    sprintf(buffer, "Timeout in trying to connect to GPS at %u baud", baudrate);
    publish_event (STS_ESP32, SS_GPS, EVENT_WARNING, buffer);
    return false;
  }
  else {
    return false;
  }
}

void gps_configure() {
  const char disableRMC[] PROGMEM = "PUBX,40,RMC,0,0,0,0,0,0";
  const char disableGLL[] PROGMEM = "PUBX,40,GLL,0,0,0,0,0,0";
  const char disableGSV[] PROGMEM = "PUBX,40,GSV,0,0,0,0,0,0";
  const char disableGST[] PROGMEM = "PUBX,40,GST,0,0,0,0,0,0";
  const char disableGSA[] PROGMEM = "PUBX,40,GSA,0,0,0,0,0,0";
  const char disableGGA[] PROGMEM = "PUBX,40,GGA,0,0,0,0,0,0";
  const char disableVTG[] PROGMEM = "PUBX,40,VTG,0,0,0,0,0,0";
  const char disableZDA[] PROGMEM = "PUBX,40,ZDA,0,0,0,0,0,0";
  const char enableRMC[] PROGMEM = "PUBX,40,RMC,1,1,1,1,1,0";
  const char enableGLL[] PROGMEM = "PUBX,40,GLL,1,1,1,1,1,0";
  const char enableGSV[] PROGMEM = "PUBX,40,GSV,1,1,1,1,1,0";
  const char enableGST[] PROGMEM = "PUBX,40,GST,1,1,1,1,1,0";
  const char enableGSA[] PROGMEM = "PUBX,40,GSA,1,1,1,1,1,0";
  const char enableGGA[] PROGMEM = "PUBX,40,GGA,1,1,1,1,1,0";
  const char enableVTG[] PROGMEM = "PUBX,40,VTG,1,1,1,1,1,0";
  const char enableZDA[] PROGMEM = "PUBX,40,ZDA,1,1,1,1,1,0";
  const char ubxAirborne[] PROGMEM = { 0x06,0x24,0x24,0x00,0x05,0x00,0x06,0x02,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0 };
  const uint32_t COMMAND_DELAY = 250; 

  // Disable unnecessary GPS messages (needed: GSA, GST, GGA, VTG) // TODO: check what can go ...
  //gps.send_P( &SerialGPS, (const __FlashStringHelper *) disableRMC ); delay( COMMAND_DELAY ); // position, velocity and time
  //gps.send_P( &SerialGPS, (const __FlashStringHelper *) disableGLL ); delay( COMMAND_DELAY ); // position data: position fix, time of position fix, and status
  //gps.send_P( &SerialGPS, (const __FlashStringHelper *) disableGSV ); delay( COMMAND_DELAY ); // number of SVs in view, PRN, elevation, azimuth, and SNR
  //gps.send_P( &SerialGPS, (const __FlashStringHelper *) disableGST ); delay( COMMAND_DELAY ); // GPS Pseudorange Noise Statistics
  //gps.send_P( &SerialGPS, (const __FlashStringHelper *) disableGSA ); delay( COMMAND_DELAY ); // GPS DOP and active satellites
  //gps.send_P( &SerialGPS, (const __FlashStringHelper *) disableGGA ); delay( COMMAND_DELAY ); // time, position, and fix related data
  //gps.send_P( &SerialGPS, (const __FlashStringHelper *) disableVTG ); delay( COMMAND_DELAY ); // actual track made good and speed over ground
  //gps.send_P( &SerialGPS, (const __FlashStringHelper *) disableZDA ); delay( COMMAND_DELAY ); // UTC day, month, and year, and local time zone offset
  gps.send_P( &SerialGPS, (const __FlashStringHelper *) enableRMC ); delay( COMMAND_DELAY ); // position, velocity and time
  gps.send_P( &SerialGPS, (const __FlashStringHelper *) enableGLL ); delay( COMMAND_DELAY ); // position data: position fix, time of position fix, and status
  gps.send_P( &SerialGPS, (const __FlashStringHelper *) enableGSV ); delay( COMMAND_DELAY ); // number of SVs in view, PRN, elevation, azimuth, and SNR
  gps.send_P( &SerialGPS, (const __FlashStringHelper *) enableGSA ); delay( COMMAND_DELAY ); // GPS DOP and active satellites
  gps.send_P( &SerialGPS, (const __FlashStringHelper *) enableGST ); delay( COMMAND_DELAY ); // GPS Pseudorange Noise Statistics
  gps.send_P( &SerialGPS, (const __FlashStringHelper *) enableGGA ); delay( COMMAND_DELAY ); // time, position, and fix related data
  gps.send_P( &SerialGPS, (const __FlashStringHelper *) enableVTG ); delay( COMMAND_DELAY ); // actual track made good and speed over ground
  gps.send_P( &SerialGPS, (const __FlashStringHelper *) enableZDA ); delay( COMMAND_DELAY ); // UTC day, month, and year, and local time zone offset  
  
  // Set navigation engine settings: CFG-NAV5 dynModel:6 fixMode:2
  // !UBX CFG-NAV5 5 6 2 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0
  sendUBX (ubxAirborne, sizeof(ubxAirborne));
  delay( COMMAND_DELAY );
  
  // Set GPS sample rate
  set_gps_samplerate (config_esp32.gps_rate);
}

void set_gps_samplerate (uint8_t rate) {
  const char ubxRate1Hz[] PROGMEM =  { 0x06,0x08,0x06,0x00,0xE8,0x03,0x01,0x00,0x01,0x00 };
  const char ubxRate5Hz[] PROGMEM =  { 0x06,0x08,0x06,0x00,0xC8,0x00,0x01,0x00,0x01,0x00 };
  const char ubxRate10Hz[] PROGMEM = { 0x06,0x08,0x06,0x00,0x64,0x00,0x01,0x00,0x01,0x00 };
  const char ubxRate16Hz[] PROGMEM = { 0x06,0x08,0x06,0x00,0x3E,0x00,0x01,0x00,0x01,0x00 };  
  switch (rate) {
    case 1:  sendUBX( ubxRate1Hz, sizeof(ubxRate1Hz) ); break;
    case 5:  sendUBX( ubxRate5Hz, sizeof(ubxRate5Hz) ); break;
    case 10: sendUBX( ubxRate10Hz, sizeof(ubxRate10Hz) ); break;
    case 16: sendUBX( ubxRate16Hz, sizeof(ubxRate16Hz) ); break;
    default: sprintf (buffer, "Invalid sample rate request for GPS (%d Hz), setting to 1 Hz", config_esp32.gps_rate);
             publish_event (STS_ESP32, SS_GPS, EVENT_WARNING, buffer);
             config_esp32.gps_rate = 1;
             sendUBX( ubxRate1Hz, sizeof(ubxRate1Hz) );
             break;
  }
}

bool set_gps_baudrate () {
  const char baud4800  [] PROGMEM = "PUBX,41,1,3,3,4800,0";
  const char baud9600  [] PROGMEM = "PUBX,41,1,3,3,9600,0";
  const char baud19200 [] PROGMEM = "PUBX,41,1,3,3,19200,0";
  const char baud38400 [] PROGMEM = "PUBX,41,1,3,3,38400,0";
  const char baud57600 [] PROGMEM = "PUBX,41,1,3,3,57600,0";
  const char baud115200[] PROGMEM = "PUBX,41,1,3,3,115200,0";
  
  switch (GPSBaud) {
    case 4800:    gps.send_P( &SerialGPS, (const __FlashStringHelper *) baud4800 ); break;
    case 9600:    gps.send_P( &SerialGPS, (const __FlashStringHelper *) baud9600 ); break;
    case 19200:   gps.send_P( &SerialGPS, (const __FlashStringHelper *) baud19200 ); break;
    case 38400:   gps.send_P( &SerialGPS, (const __FlashStringHelper *) baud38400 ); break;
    case 57600:   gps.send_P( &SerialGPS, (const __FlashStringHelper *) baud57600 ); break;
    case 115200:  gps.send_P( &SerialGPS, (const __FlashStringHelper *) baud115200 ); break;
    default:      sprintf (buffer, "Invalid baud rate request for GPS (%d baud)", GPSBaud);
                  publish_event (STS_ESP32, SS_GPS, EVENT_WARNING, buffer);
                  return false;
                  break;
  }
  // Set GPS rate to GPSBaud
  sprintf (buffer, "Setting GPS to %u baud", GPSBaud);
  publish_event (STS_ESP32, SS_GPS, EVENT_INIT, buffer);
  return true;
}

bool gps_restart() {
  const char ubxReset[] PROGMEM = { 0x06,0x04,0x00,0x00,0x01,0x00 };

  sendUBX (ubxReset, sizeof(ubxReset));
  delay (1000);
  SerialGPS.begin(GPSBaud, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
  SerialGPS.flush();
  return true;
}

bool gps_check() {
  while (gps.available(SerialGPS)) {
    esp32.gps_active = true;
    radio.gps_active = true;
    fix = gps.read();
    if (fix.status) {
      neo6mv2.millis = millis();
      neo6mv2.status = fix.status;
      neo6mv2.satellites = fix.satellites;
      if ((neo6mv2.time_valid = fix.valid.time)) {
        neo6mv2.hours = fix.dateTime.hours;
        neo6mv2.minutes = fix.dateTime.minutes;
        neo6mv2.seconds = fix.dateTime.seconds;
        neo6mv2.centiseconds = fix.dateTime_cs;
        if (!tm_this->time_set) {
          datetime.setDateTime(2000+fix.dateTime.year, fix.dateTime.month, fix.dateTime.date, fix.dateTime.hours, fix.dateTime.minutes, fix.dateTime.seconds);
          config_this->boot_epoch = datetime.getUnix()-millis()/1000;
          tm_this->time_set = true;
          sprintf (buffer, "Set time to %04u-%02u-%02u %02u:%02u:%02u based on GPS", 2000+fix.dateTime.year, fix.dateTime.month, fix.dateTime.date, fix.dateTime.hours, fix.dateTime.minutes, fix.dateTime.seconds);
          publish_event (STS_THIS, SS_THIS, EVENT_INIT, buffer);
          if (tm_this->fs_enabled) {
            create_today_dir (FS_LITTLEFS);
          }
        }
      }
      if ((neo6mv2.location_valid = fix.valid.location)) {
        neo6mv2.latitude = fix.latitudeL();
        neo6mv2.longitude = fix.longitudeL();
      }
      if ((neo6mv2.altitude_valid = fix.valid.altitude)) {
        neo6mv2.altitude = fix.altitude_cm();
        if (esp32.opsmode == MODE_CHECKOUT) {
          neo6mv2.offset_valid = neo6mv2_zero_position (neo6mv2.latitude, neo6mv2.longitude, neo6mv2.altitude);
        }
      }
      if (neo6mv2.offset_valid) {
        neo6mv2.x = (int16_t)((neo6mv2.latitude - neo6mv2.latitude_zero)*latitude_deg_to_m)/10000; //cm
        neo6mv2.y = (int16_t)((neo6mv2.longitude - neo6mv2.longitude_zero)*longitude_deg_to_m)/10000; //cm
        neo6mv2.z = (int16_t)(neo6mv2.altitude - neo6mv2.altitude_zero);
      }      
      if ((neo6mv2.speed_valid = fix.valid.velned)) {
        neo6mv2.v_north = fix.velocity_north; // cm/s
        neo6mv2.v_east = fix.velocity_east;   // cm/s
        neo6mv2.v_down = fix.velocity_down;   // cm/s
      }
      if ((neo6mv2.hdop_valid = fix.valid.hdop)) { 
        neo6mv2.milli_hdop = fix.hdop; 
      } 
      if ((neo6mv2.vdop_valid = fix.valid.vdop)) { 
        neo6mv2.milli_vdop = fix.vdop; 
      } 
      if ((neo6mv2.pdop_valid = fix.valid.pdop)) { 
        neo6mv2.milli_pdop = fix.pdop; 
      } 
      if ((neo6mv2.error_valid = (fix.valid.lat_err and fix.valid.lon_err and fix.valid.alt_err))) {
        neo6mv2.x_err = fix.lat_err_cm;
        neo6mv2.y_err = fix.lon_err_cm;
        neo6mv2.z_err = fix.alt_err_cm;
      } 
      return true;
    }
  }
  return false;
}

bool neo6mv2_zero_position (int32_t new_latitude, int32_t new_longitude, int32_t new_altitude) {
  static int32_t latitude_buffer[GPS_BUFFER_SIZE];
  static int32_t longitude_buffer[GPS_BUFFER_SIZE];
  static int32_t altitude_buffer[GPS_BUFFER_SIZE];
  static int64_t latitude_sum;
  static int64_t longitude_sum;
  static int32_t altitude_sum;
  static uint8_t gps_buffer_pos;
  static bool zero_position_ok;
  latitude_sum = latitude_sum - latitude_buffer[gps_buffer_pos] + new_latitude;
  longitude_sum = longitude_sum - longitude_buffer[gps_buffer_pos] + new_longitude;
  altitude_sum = altitude_sum - altitude_buffer[gps_buffer_pos] + new_altitude;
  latitude_buffer[gps_buffer_pos] = new_latitude;
  longitude_buffer[gps_buffer_pos] = new_longitude;
  altitude_buffer[gps_buffer_pos] = new_altitude;
  if (++gps_buffer_pos == GPS_BUFFER_SIZE) {
    gps_buffer_pos = 0;
    zero_position_ok = true;
  }
  if (zero_position_ok) {
    neo6mv2.latitude_zero = latitude_sum / GPS_BUFFER_SIZE;
    neo6mv2.longitude_zero = longitude_sum / GPS_BUFFER_SIZE;
    neo6mv2.altitude_zero = altitude_sum / GPS_BUFFER_SIZE;
    return (true);
  }
  else {
    return (false);
  }
}

void publish_udp_gps() {
  static char gps_buffer[80];
  static uint8_t gps_buffer_pos;
  while(SerialGPS.available()) {
    char c = SerialGPS.read();
    if (c != 0x0D) {
      gps_buffer[gps_buffer_pos++] = c;
    }
    if (c == 0x0A) {
      c = 0x00;
      publish_udp_text (gps_buffer);
      gps_buffer_pos = 0;
    }
  } 
}

// support functions

void sendUBX( const char *progmemBytes, size_t len ) {
  SerialGPS.write( 0xB5 ); // SYNC1
  SerialGPS.write( 0x62 ); // SYNC2
  uint8_t a = 0, b = 0;
  while (len-- > 0) {
    uint8_t c = pgm_read_byte( progmemBytes++ );
    a += c;
    b += a;
    SerialGPS.write( c );
  }
  SerialGPS.write( a ); // CHECKSUM A
  SerialGPS.write( b ); // CHECKSUM B
} 

#endif // GPS
