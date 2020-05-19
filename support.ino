/*
 * Fli3d - Various support functions
 */

void Serial_print_hexchar_bin (char ch) {
  uint8_t value;
    if (ch >= '0' && ch <= '9')
        value = ch - '0';
    if (ch >= 'A' && ch <= 'F')
        value = ch - 'A' + 10;
    if (ch >= 'a' && ch <= 'f')
        value = ch - 'a' + 10;
    if (value < 2) {
      Serial.print ("000");
    } 
    else if (value < 4) {
      Serial.print ("00");
    } 
    else if (value < 8) {
      Serial.print ("0");
    }
    Serial.print (value, BIN);  
}

void Serial_print_uint_bin (uint8_t value) {
  if (value < 2) {
    Serial.print ("0000000");
  } 
  else if (value < 4) {
    Serial.print ("000000");
  } 
  else if (value < 8) {
    Serial.print ("00000");
  }
  else if (value < 16) {
    Serial.print ("0000");
  }
  else if (value < 32) {
    Serial.print ("000");
  }
  else if (value < 64) {
    Serial.print ("00");
  }
  else if (value < 128) {
    Serial.print ("0");
  }
  Serial.println (value, BIN);  
}
