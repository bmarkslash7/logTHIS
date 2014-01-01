byte readEEPROM(int deviceaddress, unsigned int eeaddress ) {
  byte rdata = 0xFF;

  Wire.beginTransmission(deviceaddress);
  Wire.write((int)(eeaddress >> 8)); // MSB
  Wire.write((int)(eeaddress & 0xFF)); // LSB
  Wire.endTransmission();

  Wire.requestFrom(deviceaddress,1);

  if (Wire.available()) rdata = Wire.read();
  return rdata;
}


void read_eeprom()
{

  unsigned int address = 0;
  uint8_t b, bank_n=1;

#ifdef EEPROM_1M
  uint8_t bank[2] = { EEPROM_I2C_ADDR0, EEPROM_I2C_ADDR1 };
  bank_n = 2;
#elif defined EEPROM_32K
  uint8_t bank[1] = { EEPROM_I2C_ADDR0 };
#endif

  // only needs to be done once...redundant...
  Wire.begin();

  for (b=0; b < bank_n; b++)
    for (address = 0; address < (MAXADDRESS); address+=2) {
      Serial.print("[");
      Serial.print(address);
      Serial.print("] ");
      holder = ( (readEEPROM(bank[b], address)& 0xFF) << 8);
      holder |= (readEEPROM(bank[b], (address+1)) & 0xFF);
      Serial.print("0x");
      Serial.print(holder, HEX);
      Serial.print(" ");
      Serial.print(holder,DEC);
      Serial.print(" ");
      counter++;

      if ((counter == 5)) { Serial.println(); counter = 0;}
      else { Serial.print("  "); }

    }

  Serial.write("\r\n");
}

void clear_eeprom()
{
  unsigned int address = 0;
  uint8_t b, bank_n=1;

#ifdef EEPROM_1M
  uint8_t bank[2] = { EEPROM_I2C_ADDR0, EEPROM_I2C_ADDR1 };
  bank_n = 2;
#elif defined EEPROM_32K
  uint8_t bank[1] = { EEPROM_I2C_ADDR0 };
#endif

  // only needs to be done once...redundant...
  Wire.begin();

  for (b =0; b < bank_n; b++) 
  {
    Wire.beginTransmission( bank[b] );

    // start at 0
    Wire.write(0);
    Wire.write(0);

    // write with all 0xff
    for (address = 0; address < MAXADDRESS; address++)
      Wire.write(0xff);

    Wire.endTransmission();
  }


}

float convert_temperature(byte byt0, byte byt1)
{
  float f;
  f = byt0 & 0x7F;
  f *= 256;
  f += byt1;
  f /= 10;
  if (byt0 & 0x80)
    f *= -1;
  return f;
}


float convert_humidity(byte byt0, byte byt1)
{
  float f;
  f = byt0;
  f *= 256;
  f += byt1;
  f /= 10;
  return f;
}

void read_eeprom_pretty(void)
{
  char tbuf[128];
  byte byt0, byt1;
  float f;
  unsigned int address = 0;
  uint8_t b, bank_n=1;

#ifdef EEPROM_1M
  uint8_t bank[2] = { EEPROM_I2C_ADDR0, EEPROM_I2C_ADDR1 };
  bank_n = 2;
#elif defined EEPROM_32K
  uint8_t bank[1] = { EEPROM_I2C_ADDR0 };
#endif


  // only needs to be done once...redundant...
  Wire.begin();

  for (b=0; b < bank_n; b++)
  {
    for (address = 0; address < MAXADDRESS; address += 10)
    {
      sprintf(tbuf, "[%x:%4x] ", 0, address);
      Serial.write(tbuf);

      // HH:MM
      byt1 = readEEPROM(bank[b], address) & 0xff;
      byt0 = readEEPROM(bank[b], address+1) & 0xff;
      Serial.print(byt0, DEC);
      Serial.write(":");
      Serial.print(byt1, DEC);
      //Serial.write(tbuf);

      // PAR
      byt0 = readEEPROM(bank[b], address+2);
      byt1 = readEEPROM(bank[b], address+3);
      sprintf(tbuf, " PAR(%x,%x)", byt0, byt1);
      Serial.write(tbuf);

      // PAR CAP
      byt0 = readEEPROM(bank[b], address+4);
      byt1 = readEEPROM(bank[b], address+5);
      sprintf(tbuf, " CAP(%x,%x)", byt0, byt1);
      Serial.write(tbuf);

      // Temp
      f = convert_temperature(
            readEEPROM(bank[b], address+4),
            readEEPROM(bank[b], address+5) );
      sprintf(tbuf, " T(%f C)", (double)f);
      Serial.write(tbuf);

      // Humidity
      f = convert_humidity(
            readEEPROM(bank[b], address+4),
            readEEPROM(bank[b], address+5) );
      sprintf(tbuf, " Hum(%f)", (double)f);
      Serial.write(tbuf);

      Serial.write("\r\n");
    }
  }

}
