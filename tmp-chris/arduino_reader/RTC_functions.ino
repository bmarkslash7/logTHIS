void read_rtc_date()
{
  uint8_t i;
  byte byt;

  Serial.write("\n\r");

  // redundant
  Wire.begin();

  Wire.beginTransmission(RTC_I2C_ADDR);
  Wire.write(0);
  Wire.endTransmission();
  Wire.requestFrom(RTC_I2C_ADDR,16);

  for (i=0; i<16; i++)
  {
    if (Wire.available()) 
    {
      byt = Wire.read();
      Serial.print(byt);

      if (i == 8) 
        Serial.write("\n\r");
    }
  }

}

void read_rtc_date_pretty()
{
  uint8_t i;
  byte byt;

  Serial.write("\n\r");

  // redundant
  Wire.begin();

  Wire.beginTransmission(RTC_I2C_ADDR);
  Wire.write(0);
  Wire.endTransmission();
  Wire.requestFrom(RTC_I2C_ADDR,16);
  for (i=0; i<16; i++)
  {
    if (Wire.available()) 
    {
      byt = Wire.read();
      Serial.print(byt, HEX);
      Serial.write(" ");

      if (i == 8) 
        Serial.write("\n\r");
    }
  }

}

void enter_rtc_time() {
  char minutes[2], hour[2], day[2], date[2], year[4];
  char tempint;
  Serial.write("set_rtc_date not implemnted\r\n");
  while(Serial.available() > 0) Serial.read(); //Should flush the buffer
  Serial.write("Please enter the minutes with two numbers (1 is 01)\n");
  while (!Serial.available());
  Serial.readBytes(minutes, 2);
  //minutes = tempint - '0';
  Serial.println(minutes);
  
  Serial.write("Please enter the hour in 24 hour format\n");
  while (!Serial.available());
  Serial.readBytes(hour, 2);
//  hour = tempint - '0';
  Serial.println(hour);
  
 // set_rtc_date(0, minutes, hour, day, date, year);
}

void set_rtc_date(byte sec, byte minutes, byte hour, byte day, byte date, int year)
{
  byte year_l = 0xff, year_h = 0xff;
  
  
  // redundant
  Wire.begin();


  //DOES NOT WORK
  // just a placeholder for now
  Wire.beginTransmission(RTC_I2C_ADDR);
  Wire.write(0);
  
  Wire.write(sec);
  Wire.write(minutes);
  Wire.write(hour);
  Wire.write(day);
  Wire.write(date);
  Wire.write(year_h);
  Wire.write(year_l);
  Wire.endTransmission();

}
