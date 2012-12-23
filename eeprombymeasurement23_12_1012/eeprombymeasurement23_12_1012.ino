/* ****************************************************
code taken from http://www.insidegadgets.com/2010/12/22/reading-data-from-eeprom-i2c-on-a-pcb/
need to rewrite to store bytes adn print in coherent fashion
******************************************************** */

#include <Wire.h>
 
#define disk1 0x50
#define MAXADDRESS 4096//4096
unsigned int holder = 0;
int counter = 0;
 
void setup(void){
 Serial.println("My DataLogger");
 Serial.begin(9600);
 Wire.begin();
 unsigned int address = 0;
 for (address = 0; address < (MAXADDRESS); address+=2) {
   Serial.print(address);
   Serial.print("\t");
   //Serial.print(readEEPROM(disk1, address), HEX);
   //Serial.print(readEEPROM(disk1, (address+1)), HEX);
   //Serial.print("\t");
   holder = 0;
   holder = ( (readEEPROM(disk1, address)& 0xFF) << 8);
//   delay(5);
   holder |= (readEEPROM(disk1, (address+1)) & 0xFF);
   Serial.print(holder, HEX);
   Serial.print("\t");
   Serial.print(holder,DEC);
   counter++;
   
   if ((counter == 5)) { Serial.println(); counter = 0;}
   else { Serial.print("\t"); }

 }
}
 
void loop(){
 
}
 
byte readEEPROM(int deviceaddress, unsigned int eeaddress ) {
 byte rdata = 0xFF;
 
 Wire.beginTransmission(deviceaddress);
 Wire.write((int)(eeaddress >> 8));   // MSB
 Wire.write((int)(eeaddress & 0xFF)); // LSB
 Wire.endTransmission();
 
 Wire.requestFrom(deviceaddress,1);
 
 if (Wire.available()) rdata = Wire.read();
 return rdata;
}
