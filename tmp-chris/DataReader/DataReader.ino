/* DataReader Code
1) Upload to an Arduino
2) Start a serial monitor
3) Type in commands to modify logTHIS

Read/Write external EEPROM
Read/Write Real Time Clock

TODO: Update read and write to include both pages of the memory!
TODO: Update MAXADDRESS to be actual maximum address
*/
#include <stdlib.h>
#include <Wire.h>
#define disk1 0x50
#define rtc 0x00
#define MAXADDRESS 4095

String inputString = "";
bool stringComplete = false;

void setup(void) {
  Serial.begin(9600);
  Serial.println("logTHIS DataReader");

  pinMode(4, OUTPUT);
  pinMode(5, OUTPUT);
  pinMode(6, OUTPUT);
  pinMode(7, OUTPUT);

}


void loop(void) {
  stringComplete = false;
  inputString == "";
  
  digitalWrite(4, LOW);
  digitalWrite(5, LOW);
  digitalWrite(6, LOW);
  digitalWrite(7, LOW);
  
  Serial.println("What would you like to do?  Pick a number.");
  Serial.println("1. Read the time on the clock");
  Serial.println("2. Write the current time to the clock");
  Serial.println("3. Download data");
  Serial.println("4. Erase data from datalogger");
  
  Serial.flush();
  while (!Serial.available());//stringComplete == true) {delay(100);}
  
  char tempint;
  tempint = Serial.read();
//  Serial.println(tempint);
 int choice = tempint - '0';
//  Serial.print("choice is: ");
//  Serial.println(choice);
    switch ( choice ) {
    case 1:
    digitalWrite(4,HIGH);
    
//      readClock(rtc);
      break;
    case 2:
    digitalWrite(5,HIGH);
//      writeClock(rtc);
      break;
    case 3:
    digitalWrite(6,HIGH);
//      readEEPROM(disk1);
      break;
    case 4:
    digitalWrite(7,HIGH);
/*      Serial.println("Are you sure? 1.Y or 2.N (default is N) Type in Number");
      if (Serial.available() > 0) {
        int sure = Serial.read();
        if (sure == 1) {eraseEEPROM(disk1);}
        else if (sure == 2) {Serial.println("Date were not erased");}
        else {Serial.println("Invalid response.  Date were not erased");}
 */
       break;
    default:
      Serial.println("Not a valid choice.  Please pick again");
      break;
    }    
//  }
 delay(5000);
  
}

void serialEvent() {
 while (Serial.available() ) {
  char inChar = (char)Serial.read();
  inputString += inChar;
 if (inChar == '\n') {stringComplete = true;}
 } 
}

void readEEPROM(int deviceaddress) {
/* ****************************************************
code taken from http://www.insidegadgets.com/2010/12/22/reading-data-from-eeprom-i2c-on-a-pcb/
need to rewrite to store bytes adn print in coherent fashion
******************************************************** */
  
   Wire.begin();
 unsigned int address = 0;
 int counter = 0;
 for (address = 0; address < (MAXADDRESS); address+=2) {
   Serial.print(address);
   Serial.print("\t");
   //Serial.print(readEEPROM(disk1, address), HEX);
   //Serial.print(readEEPROM(disk1, (address+1)), HEX);
   //Serial.print("\t");
   int holder = 0;
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

void eraseEEPROM(int deviceaddress) {
 Wire.begin();
 unsigned int address = 0;
 
 for (address = 0; address < MAXADDRESS; address++)
 {
   writeEEPROM(deviceaddress, address, 0x00);
   delay(5);
 }
 delay(10);
 Serial.println("External EEPROM has been erased.");
}

byte readEEPROM(int deviceaddress, unsigned int eeaddress ) {
 byte rdata = 0xFF;
 
 Wire.beginTransmission(deviceaddress);
 Wire.write((int)(eeaddress >> 8) & 0x0F);   // MSB
 Wire.write((int)(eeaddress & 0xFF)); // LSB
 Wire.endTransmission();
 
 Wire.requestFrom(deviceaddress,1);
 
 if (Wire.available()) rdata = Wire.read();
 return rdata;
}

byte writeEEPROM(int deviceaddress, unsigned int eeaddress, byte value)
{
  int rdata = value;
  Wire.beginTransmission(deviceaddress);
  Wire.write( ( (byte) (eeaddress >> 8)) );
  Wire.write( (byte) (eeaddress & 0xFF) );
  Wire.write(rdata);
  Wire.endTransmission();
}
