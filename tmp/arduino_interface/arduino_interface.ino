#include <Wire.h>

#include "menu_system.h"

//#define EEPROM_32K
#define EEPROM_1M


#define EEPROM_I2C_ADDR0 0x50
#ifdef EEPROM_1M
  #define EEPROM_I2C_ADDR1 0x54 
#endif

#ifdef EEPROM_1M
  #define MAXADDRESS_foo 64000
#elif defined EEPROM_32K
  #define MAXADDRESS_foo 4000
#endif

#define RTC_I2C_ADDR 0x68


//#define MAXADDRESS 4096
#define MAXADDRESS 240



unsigned int holder = 0;
int counter = 0;

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


void set_rtc_date(byte sec, byte min, byte hour, byte day, byte date, int year)
{
  byte year_l = 0xff, year_h = 0xff;
  Serial.write("set_rtc_date not implemnted\r\n");

  // redundant
  Wire.begin();

  //DOES NOT WORK
  // just a placeholder for now
  Wire.beginTransmission(RTC_I2C_ADDR);
  Wire.write(0);
  Wire.write(sec);
  Wire.write(min);
  Wire.write(hour);
  Wire.write(day);
  Wire.write(date);
  Wire.write(year_h);
  Wire.write(year_l);
  Wire.endTransmission();

}

void setup() {
  // initialize serial:
  Serial.begin(9600);
  Serial.write("$ ");
  cmd_init();
}

void loop() {
  char byt;

  // if there's any serial available, read it:
  while (Serial.available() > 0) {
    byt = Serial.read();
    cmd_process(byt);
  }

}


//------------------------ menu system



cmd_t cmd = {0};

void cmd_history_commit()
{
  int i, j, k, pos;

  // get destination position into static history to copy
  // current command into
  pos = ( cmd.history_start + cmd.history_n ) % CMD_HISTORY_LEN;
  if ( cmd.history_n < CMD_HISTORY_LEN )
    cmd.history_n ++;

  if (cmd.history_n == CMD_HISTORY_LEN)
    cmd.history_start = (cmd.history_start + 1) % CMD_HISTORY_LEN;

  // copy current command into static history
  k = cmd.history_scratch_pos;
  for (i=0; i < CMD_BUF_LEN; i++)
    cmd.history[ pos ][i] = cmd.history_scratch[ k ][i];

  // copy static history into scratch
  for (i=0; i < CMD_HISTORY_LEN; i++)
    for (j=0; j < CMD_BUF_LEN; j++)
      cmd.history_scratch[i][j] = cmd.history[i][j];

  // update scratch position
  cmd.history_scratch_pos = (cmd.history_start + cmd.history_n) % CMD_HISTORY_LEN;

  // keep buf as our pointer to our active command line to simplify things
  cmd.buf = cmd.history_scratch[ cmd.history_scratch_pos ];
  cmd.buf[0] = '\0';
  cmd.buf_pos = 0;
  cmd.buf_n = 0;


  Serial.write("$ ");

}

void cmd_verbose_print(void)
{
  int i, p;
  char tbuf[128];

  sprintf(tbuf, "cmd history_start %i, history_n %i\n\r",
    cmd.history_start, cmd.history_n);
  Serial.write(tbuf);

  Serial.write("history:\r\n");
  for (i=0; i<cmd.history_n; i++)
  {
    p = (cmd.history_start + i) % CMD_HISTORY_LEN;
    sprintf(tbuf, "  [%i] '%s'\r\n", p, cmd.history[p]);
    Serial.write(tbuf);
  }

  sprintf(tbuf, "\r\nhistory_scratch (%i):\r\n", cmd.history_scratch_pos);
  Serial.write(tbuf);
  for (i=0; i<cmd.history_n; i++)
  {
    p = (cmd.history_start + i) % CMD_HISTORY_LEN;
    sprintf(tbuf, " %c[%i] '%s'\r\n", (p == cmd.history_scratch_pos) ? '*' : ' ', p, cmd.history_scratch[p]);
    Serial.write(tbuf);
  }


  sprintf(tbuf, "buf_n %i, buf_pos %i '%s'\r\n", cmd.buf_n, cmd.buf_pos, cmd.buf);
  Serial.write(tbuf);
  sprintf(tbuf, "in_esc %i, esc_buf(%i,%i,%i) esc_buf_n %i\n\r",
      cmd.in_esc, cmd.esc_buf[0], cmd.esc_buf[1], cmd.esc_buf[2], cmd.esc_buf_n);
  Serial.write(tbuf);
  Serial.write("\r\n");

}

//--------------

void cmd_bs()
{
  int i;

  if (cmd.buf_pos == 0)
    return;

  Serial.write("\b");


  for ( i = cmd.buf_pos; i < cmd.buf_n; i++)
  {
    cmd.buf[i-1] = cmd.buf[i];

    Serial.write("\b"); //????
  }

  cmd.buf_pos--;
  cmd.buf[--cmd.buf_n] = '\0';

  Serial.write(" ");

  for ( i = cmd.buf_n; i > cmd.buf_pos; i-- )
    Serial.write("\b");

  Serial.write("\b");

}

//--------------

// not implemented
void cmd_too_long(uint8_t byt)
{
}

//--------------

void cmd_add_byte(uint8_t byt)
{
  int i;

  // just in case
  if (cmd.buf_n >= (CMD_BUF_LEN-1)) 
    return;

  Serial.write(&byt, 1);  //???

  // shift over latter characters
  for (i = cmd.buf_pos; i < cmd.buf_n; i++)
    Serial.write(cmd.buf + i , 1); //????

  // move cursor back
  for (i = cmd.buf_pos; i < cmd.buf_n; i++)
    Serial.write("\b"); 

  // copy over characters in our buffer
  for (i = cmd.buf_n; i > cmd.buf_pos; i--)
    cmd.buf[i] = cmd.buf[i-1];
  cmd.buf[cmd.buf_pos++] = byt;
  cmd.buf[++cmd.buf_n] = '\0';

  if (cmd.buf_pos == (CMD_BUF_LEN-1))
  {
    Serial.write("\b"); //????
    cmd.buf_pos--;
  }


}

//--------------

void cmd_cursor_left(void)
{
  if (cmd.buf_pos == 0) return;
  cmd.buf_pos--;

  Serial.write("\b");
}

//--------------

void cmd_cursor_right(void)
{
  if (cmd.buf_pos == cmd.buf_n ) return;
  cmd.buf_pos++;

  Serial.write(cmd.buf + cmd.buf_pos - 1, 1); //????
}

//--------------

void cmd_cursor_up(void)
{

  int i, pos;

  // check if we've gotten to the first position
  pos = (cmd.history_scratch_pos + 8 - 1) % CMD_HISTORY_LEN;
  if ((cmd.history_n == CMD_HISTORY_LEN) && (pos == cmd.history_start))
    return;
  if (pos > cmd.history_n)
    return;

  // erase our current command from the terminal
  for (i=cmd.buf_pos; i<cmd.buf_n; i++)
    Serial.write(" ");
  for (i=0; i<cmd.buf_n; i++)
    Serial.write("\b \b");

  // update our buffer
  cmd.buf = cmd.history_scratch[ pos ];
  cmd.buf_n = strlen((char *)cmd.history_scratch[ pos ]);
  cmd.history_scratch_pos = pos;

  cmd.buf_pos = cmd.buf_n;

  // print out our new command
  if (cmd.buf_n > 0)
    Serial.print((char *)cmd.buf); //????

  // repositino our cursor (currently not implemented properly)
  for (i=cmd.buf_pos ; i < cmd.buf_n; i++)
    Serial.write("\b");

}

//--------------

void cmd_cursor_down()
{
  int i, pos, b;

  // make sure we haven't fallen off of the edge
  b = (cmd.history_start + cmd.history_n + 1) % CMD_HISTORY_LEN;
  pos = (cmd.history_scratch_pos + 1) % CMD_HISTORY_LEN;
  if (pos == b)
    return;

  // clear out command fromt he terminal
  for (i=cmd.buf_pos; i<cmd.buf_n; i++)
    Serial.write(" ");
  for (i=0; i<cmd.buf_n; i++)
    Serial.write("\b \b");

  // update our buffer
  cmd.buf = cmd.history_scratch[ pos ];
  cmd.buf_n = strlen((char *)cmd.history_scratch[ pos ]);
  cmd.history_scratch_pos = pos;

  cmd.buf_pos = cmd.buf_n;

  if (cmd.buf_n > 0)
    Serial.print((char *)cmd.buf);

  for (i=cmd.buf_pos ; i < cmd.buf_n; i++)
    Serial.write("\b");

}

//--------------

void cmd_init()
{
  cmd.history_n=0;
  cmd.buf = cmd.history_scratch[0];
}

void cmd_show_help(void)
{
  Serial.write("help\n\r");
  Serial.write("p    program\n\r");
  Serial.write("e    read eeprom\n\r");
  Serial.write("E    read eeprom (pretty)\n\r");
  Serial.write("w    write eeprom\n\r");
  Serial.write("d    get date\n\r");
  Serial.write("s    set date\n\r");
  Serial.write("R    reset\n\r");
  Serial.write("h/?  help\n\r");
  Serial.write("~    DEBUG show current command history\n\r");
  Serial.write("v    DEBUG show command history\n\r");

  fflush(stdout);

}

void cmd_exec(uint8_t *b)
{

  Serial.write("\r\n");

  if (b[0] == 'p')
  {
    Serial.write("program avr not implemented\r\n");
  }
  else if (b[0] == 'e')
  {
    read_eeprom();
  }
  else if (b[0] == 'E')
  {
    read_eeprom_pretty();
  }
  else if (b[0] == 'w')
  {
    Serial.write("write eeprom not implemented\n\r");
  }
  else if (b[0] == 'd')
  {
    Serial.write("get date not implemented\n\r");
  }
  else if (b[0] == 's')
  {
    Serial.write("set date not implemented\n\r");
  }
  else if (b[0] == 'R')
  {
    Serial.write("reset not implemented\n\r");
  }
  else if (b[0] == 'v')
  {
    cmd_verbose_print();
  }
  else if ((b[0] == '\0') || (b[0] == '\r') || (b[0] == '\n'))
  {
  }
  else
  {
    if ((b[0] != 'h') && (b[0] != '?'))
      Serial.write("invalid command\n\r");
    cmd_show_help();
  }

  fflush(stdout);

}


//-----------------------------------------------


void cmd_handle_esc(uint8_t byt)
{

  cmd.esc_buf[ cmd.esc_buf_n++ ] = byt;
  if (cmd.esc_buf_n == 3)
  {

    if (cmd.esc_buf[1] == '[') 
    {
      if (cmd.esc_buf[2] == 'A')
      {
        cmd_cursor_up();
      }
      else if (cmd.esc_buf[2] == 'B')
      {
        cmd_cursor_down();
      }
      else if (cmd.esc_buf[2] == 'C')
      {
        cmd_cursor_right();
      }
      else if (cmd.esc_buf[2] == 'D')
      {
        cmd_cursor_left();
      }
    }

    cmd.in_esc = 0;
    cmd.esc_buf_n=0;

  }

}

//-----------------------------------------------

void cmd_process(char byt)
{
  //char tbuf[128];
  //sprintf(tbuf, "\r\n[%x] %i\r\n", (int)byt, (int)byt);
  //Serial.write(tbuf);
  //printf("%s", tbuf); fflush(stdout);

  // our special special character
  if (byt == '~')
  {
    Serial.write("\r\n"); 
    cmd_verbose_print();
    return;
  }

  if (cmd.in_esc)
  {
    cmd_handle_esc(byt);
    return;
  }
  else if (byt == 27) // ESC
  {
    cmd.in_esc = 1;
    cmd.esc_buf[ cmd.esc_buf_n++ ] = byt;
    return;
  }

  if (byt == BS )
  {
    cmd_bs();
    return;
  }
  else if (byt == '\r') 
  {
    cmd_exec(cmd.buf);
    cmd_history_commit();
    return;
  }
  else if (cmd.buf_n == (CMD_BUF_LEN-1))
  {
    cmd_too_long(byt);
    return;
  }

  cmd_add_byte(byt);

}


