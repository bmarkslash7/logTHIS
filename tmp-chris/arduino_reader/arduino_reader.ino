#include <Wire.h>

#include "Arduino.h"
//#include "menu_system.h"

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

#define CMD_BUF_LEN 8
#define CMD_HISTORY_LEN 8
#define BS 127

//typedef unsigned char uint8_t;

typedef struct command_type {

  uint8_t history[ CMD_HISTORY_LEN ][ CMD_BUF_LEN ];
  uint8_t history_start;
  uint8_t history_n;

  uint8_t history_scratch[ CMD_HISTORY_LEN ][ CMD_BUF_LEN ];
  uint8_t history_scratch_pos;

  uint8_t *buf;
  uint8_t buf_n, buf_pos;

  uint8_t in_esc;
  uint8_t esc_buf[3];
  uint8_t esc_buf_n;
} cmd_t;


unsigned int holder = 0;
int counter = 0;

void setup() {
  // initialize serial:
  Serial.begin(9600);
  Serial.write("$ ");
  
  cmd_init();
  cmd_show_help();
}

void loop() {
  char byt;

  // if there's any serial available, read it:
  while (Serial.available() > 0) {
    byt = Serial.read();
    //cmd_process(byt);
    
    // original cmd_exec appears to have a mismatch with types
// when entering a command and hitting enter, nothing would occur
// Quick work around is to call cmd_exec_chris with a char pointer
    cmd_exec_chris(&byt);
  }

}




