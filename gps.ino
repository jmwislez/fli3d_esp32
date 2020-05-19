// TODO: disable unnecessary messages (done?)
// TODO: check whether settings are correctly applied, issue warning if not
// TODO: check usefulness of speed
// TODO: test impact of loss or temporary interruption of communications
// TODO: absolute figure for 3D accuracy should be available

/*
 * Fli3d - gps functionality
 */
 
#ifdef GPS

#include <NMEAGPS.h> 
#include <ublox/ubxGPS.h>

#define GPSBaud 57600 // tried 115200 but got garbage in some cases
#define SerialGPS Serial2

static NMEAGPS gps;
static gps_fix fix;

bool gps_setup () {
  const unsigned char ubxRate1Hz[] PROGMEM =  { 0x06,0x08,0x06,0x00,0xE8,0x03,0x01,0x00,0x01,0x00 };
  const unsigned char ubxRate5Hz[] PROGMEM =  { 0x06,0x08,0x06,0x00,0xC8,0x00,0x01,0x00,0x01,0x00 };
  const unsigned char ubxRate10Hz[] PROGMEM = { 0x06,0x08,0x06,0x00,0x64,0x00,0x01,0x00,0x01,0x00 };
  const unsigned char ubxRate16Hz[] PROGMEM = { 0x06,0x08,0x06,0x00,0x32,0x00,0x01,0x00,0x01,0x00 };
  const unsigned char ubxAirborne[] PROGMEM = { 0x06,0x24,0x24,0x00,0x05,0x00,0x06,0x02,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0 };
  const char disableRMC[] PROGMEM = "PUBX,40,RMC,0,0,0,0,0,0";
  const char disableGLL[] PROGMEM = "PUBX,40,GLL,0,0,0,0,0,0";
  const char disableGSV[] PROGMEM = "PUBX,40,GSV,0,0,0,0,0,0";
  const char disableGSA[] PROGMEM = "PUBX,40,GSA,0,0,0,0,0,0";
  const char disableGGA[] PROGMEM = "PUBX,40,GGA,0,0,0,0,0,0";
  const char disableVTG[] PROGMEM = "PUBX,40,VTG,0,0,0,0,0,0";
  const char disableZDA[] PROGMEM = "PUBX,40,ZDA,0,0,0,0,0,0";
  const char enableRMC[] PROGMEM = "PUBX,40,RMC,1,1,1,1,1,0";
  const char enableGLL[] PROGMEM = "PUBX,40,GLL,1,1,1,1,1,0";
  const char enableGSV[] PROGMEM = "PUBX,40,GSV,1,1,1,1,1,0";
  const char enableGSA[] PROGMEM = "PUBX,40,GSA,1,1,1,1,1,0";
  const char enableGGA[] PROGMEM = "PUBX,40,GGA,1,1,1,1,1,0";
  const char enableVTG[] PROGMEM = "PUBX,40,VTG,1,1,1,1,1,0";
  const char enableZDA[] PROGMEM = "PUBX,40,ZDA,1,1,1,1,1,0";
  const char baud9600  [] PROGMEM = "PUBX,41,1,3,3,9600,0";
  const char baud38400 [] PROGMEM = "PUBX,41,1,3,3,38400,0";
  const char baud57600 [] PROGMEM = "PUBX,41,1,3,3,57600,0";
  const char baud115200[] PROGMEM = "PUBX,41,1,3,3,115200,0";
  const uint32_t COMMAND_DELAY = 250; 


  if (gps_connect (9600) or gps_connect (GPSBaud)) {
    // Disable unnecessary GPS messages (TODO: define which)
    //gps.send_P( &SerialGPS, (const __FlashStringHelper *) disableRMC ); delay( COMMAND_DELAY ); // position, velocity and time
    //gps.send_P( &SerialGPS, (const __FlashStringHelper *) disableGLL ); delay( COMMAND_DELAY ); // position data: position fix, time of position fix, and status
    //gps.send_P( &SerialGPS, (const __FlashStringHelper *) disableGSV ); delay( COMMAND_DELAY ); // number of SVs in view, PRN, elevation, azimuth, and SNR
    //gps.send_P( &SerialGPS, (const __FlashStringHelper *) disableGSA ); delay( COMMAND_DELAY ); // GPS DOP and active satellites
    //gps.send_P( &SerialGPS, (const __FlashStringHelper *) disableGGA ); delay( COMMAND_DELAY ); // time, position, and fix related data
    //gps.send_P( &SerialGPS, (const __FlashStringHelper *) disableVTG ); delay( COMMAND_DELAY ); // actual track made good and speed over ground
    //gps.send_P( &SerialGPS, (const __FlashStringHelper *) disableZDA ); delay( COMMAND_DELAY ); // UTC day, month, and year, and local time zone offset
    gps.send_P( &SerialGPS, (const __FlashStringHelper *) enableRMC ); delay( COMMAND_DELAY ); // position, velocity and time
    gps.send_P( &SerialGPS, (const __FlashStringHelper *) enableGLL ); delay( COMMAND_DELAY ); // position data: position fix, time of position fix, and status
    gps.send_P( &SerialGPS, (const __FlashStringHelper *) enableGSV ); delay( COMMAND_DELAY ); // number of SVs in view, PRN, elevation, azimuth, and SNR
    gps.send_P( &SerialGPS, (const __FlashStringHelper *) enableGSA ); delay( COMMAND_DELAY ); // GPS DOP and active satellites
    gps.send_P( &SerialGPS, (const __FlashStringHelper *) enableGGA ); delay( COMMAND_DELAY ); // time, position, and fix related data
    gps.send_P( &SerialGPS, (const __FlashStringHelper *) enableVTG ); delay( COMMAND_DELAY ); // actual track made good and speed over ground
    gps.send_P( &SerialGPS, (const __FlashStringHelper *) enableZDA ); delay( COMMAND_DELAY ); // UTC day, month, and year, and local time zone offset  
    // Set navigation engine settings: CFG-NAV5 dynModel:6 fixMode:2
    // !UBX CFG-NAV5 5 6 2 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0
    sendUBX (ubxAirborne, sizeof(ubxAirborne));
    // Set GPS sample rate
    switch (eeprom_esp32.gps_rate) {
      case 1:  sendUBX( ubxRate1Hz, sizeof(ubxRate1Hz) ); break;
      case 5:  sendUBX( ubxRate5Hz, sizeof(ubxRate5Hz) ); break;
      case 10: sendUBX( ubxRate10Hz, sizeof(ubxRate10Hz) ); break;
      case 16: sendUBX( ubxRate16Hz, sizeof(ubxRate16Hz) ); break;
      default: sprintf (buffer, "Invalid sample rate request for GPS (%d Hz); fallback to 1 Hz", eeprom_esp32.gps_rate);
               bus_publish_event (STS_ESP32, SS_NEO6MV2, EVENT_WARNING, buffer);
               eeprom_esp32.gps_rate = 1;
               break;
    }
    sprintf (buffer, "Set GPS rate to %d Hz", eeprom_esp32.gps_rate);  
    bus_publish_event (STS_ESP32, SS_NEO6MV2, EVENT_INIT, buffer);
    // Set GPS rate to GPSBaud
    switch (GPSBaud) {
      case 9600:    gps.send_P( &SerialGPS, (const __FlashStringHelper *) baud9600 ); break;
      case 38400:   gps.send_P( &SerialGPS, (const __FlashStringHelper *) baud38400 ); break;
      case 57600:   gps.send_P( &SerialGPS, (const __FlashStringHelper *) baud57600 ); break;
      case 115200:  gps.send_P( &SerialGPS, (const __FlashStringHelper *) baud115200 ); break;
      default:      sprintf (buffer, "Invalid baud rate request for GPS (%d baud); staying at 9600 baud", GPSBaud);
                    bus_publish_event (STS_ESP32, SS_NEO6MV2, EVENT_WARNING, buffer);
                    return true;
                    break;  
    }
    SerialGPS.flush();
    SerialGPS.end();   
    delay( 2*COMMAND_DELAY );
    return gps_connect (GPSBaud);
  }
  else {
    return false;
  }
}

bool gps_connect (uint16_t baud) {
  SerialGPS.begin(baud, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
  uint32_t start_probe_millis = millis ();
  char gps_buffer[255];
  uint8_t gps_buffer_pos;
  while (millis() - start_probe_millis < 2000) {
    while (SerialGPS.available()) {
      gps_buffer[gps_buffer_pos] = SerialGPS.read();
      if (gps_buffer[gps_buffer_pos] == '\n') {
        // proper data received
        sprintf (buffer, "Connected to GPS at %d baud", baud);
        bus_publish_event (STS_ESP32, SS_NEO6MV2, EVENT_INIT, buffer);
        return true;
      }
      if (gps_buffer_pos == 250) {
        sprintf (buffer, "Garbage received over serial from GPS (at %d baud); disabling", baud); 
        bus_publish_event (STS_ESP32, SS_NEO6MV2, EVENT_ERROR, buffer);
        return false;
      }
      gps_buffer_pos++;
    }
  }
  // timeout
  sprintf (buffer, "No serial traffic detected from GPS (at %d baud); disabling", baud); 
  bus_publish_event (STS_ESP32, SS_NEO6MV2, EVENT_ERROR, buffer);
  return false;
}

bool gps_check () {
  while (gps.available(SerialGPS)) {
    esp32.gps_current = true;
    fix = gps.read();
    if (fix.status) {
      neo6mv2.millis = millis ();
      neo6mv2.status = fix.status;
      neo6mv2.satellites = fix.satellites;
      if (neo6mv2.time_valid = fix.valid.time) {
        neo6mv2.hours = fix.dateTime.hours;
        neo6mv2.minutes = fix.dateTime.minutes;
        neo6mv2.seconds = fix.dateTime.seconds;
      }
      if (neo6mv2.location_valid = fix.valid.location) {
        neo6mv2.latitude = fix.latitude();
        neo6mv2.longitude = fix.longitude();
      }
      if (neo6mv2.altitude_valid = fix.valid.altitude) {
        neo6mv2.altitude = fix.altitude();
      }
      if (neo6mv2.speed_valid = fix.valid.speed) {
        neo6mv2.v_north = fix.velocity_north;
        neo6mv2.v_east = fix.velocity_east;
        neo6mv2.v_down = fix.velocity_down; 
      }
      if (neo6mv2.hdop_valid = fix.valid.hdop) {
        neo6mv2.milli_hdop = fix.hdop;
      }
      if (neo6mv2.vdop_valid = fix.valid.vdop) {
        neo6mv2.milli_vdop = fix.vdop;
      }
      return true;
    }
  }
  return false;
}

bool neo6mv2_checkConfig () {
  // TODO: TBW
}

void neo6mv2_printConfig () {
  // TODO: TBW
}

// support functions

void sendUBX( const unsigned char *progmemBytes, size_t len ) {
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

#endif
