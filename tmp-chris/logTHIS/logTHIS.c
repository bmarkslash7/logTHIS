/*********************************************
ver2 of datalogger:
Wake up from alarm from RTC once a minute
Determine whether uC needs to collect data
Read from Humidity sensor (keep trying a couple of times if initial fails
Read from Light sensor - convert cap sensor to ADC
Write data to EEPROM
Put AVR to sleep
 * Author: christopher stieha
 * Filename: logTHIS.c
 * Chip: ATtiny85

I2c code from http://www.instructables.com/id/I2C_Bus_for_ATtiny_and_ATmega/ user doctek
HUMIDITY code modified from https://github.com/nethoncho/Arduino-DHT22 user nethoncho
 */
// From http://playground.arduino.cc/Main/AVR 
// use this to clear bits
#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
// Use this to set bits
#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif 

#define F_CPU 1000000
#include <avr/io.h>
#include <util/delay.h>
#include <inttypes.h>
#include <stdint.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>
#include <avr/sleep.h>
#include "USI_TWI_Master.h"

// PIN DEFINITIONS
#define PAR_SENSOR 3
#define HUMIDITY 4
#define HUMIDITY_CLOCK_TIME 6		// number compared to timing of humidity sensors to determine 0 or 1
					// changes with the speed of the processor
#define DISPLAY_LED 4

// EEPROM definitions
// #define MEMORY_ADDR  0b1010		// function needs to <<4  add BLOCK (0,1) add Chip select (two bits), and R/W 
#define MEMORY_ADDR  0b1010000		// 32kb EEPROM
#define MESSAGEBUF_SIZE       9

// RTC definitions
#define RTC_ADDR 0b1101000		// function needs to <<1 and add R/W bit

#define HOW_OFTEN 2			// 0 is every minute, 1 is once an hour, 2 is twice an hour, 
							// 4 is 4 times and hour, 6 is 6 times an hour

#define DHT22_ERROR_VALUE -99.5

// This should be 40, but the sensor is adding an extra bit at the start
#define DHT22_DATA_BIT_COUNT 41


// FUNCTION DECLARATIONS
int set_PORTB_bit (int position, int value);
int set_DDRB_bit (int position, int value);
uint16_t readPAR_Sensor (int pinPAR_SENSOR);
uint16_t readPAR_SensorADC (int pinPAR_SENSOR);
uint16_t readPAR_SensorCAPADC (int pinPAR_SENSOR);
int set_RTC_bit (uint8_t device, uint8_t address, uint8_t bit, uint8_t value);


// ------------------HERE STARTS MAIN ----------------------------------------------
int main (void)
{
// configure sleep state and interrupt pin
//	cli();
	set_sleep_mode(SLEEP_MODE_PWR_DOWN);	
	
	
// blink on and off to say it is on and working
	set_DDRB_bit(6,1);
	// Blink DISPLAY_LED on and off to show that it has started
	int blinki;
	for (blinki = 0; blinki<10; blinki++) {
		set_PORTB_bit(6,1);  // sets B0 as high
		_delay_ms(200);
		set_PORTB_bit(6,0); // sets B0 as low
		_delay_ms(200);
	}
	set_DDRB_bit(6,0);
// -------------------------------------------
	
//        sbi(GIMSK, 5);
	GIMSK |= (1 << 5);	// enable interrupts
	PCMSK |= (1 << 1);	// set Pin6/PB1/PCINT1 as interupt pin
				// currently same as DISPLAY_LED

// initialize pins
	USI_TWI_Master_Initialise();
	set_DDRB_bit(PAR_SENSOR,1);
	set_DDRB_bit(HUMIDITY,1);
	set_DDRB_bit(DISPLAY_LED,1);

// initialize variables
	uint16_t externaleepromcounter = 0;
//	uint16_t internaleepromcounter = 0;
	int i;
	
	int16_t currentHumidity;
	int16_t currentTemperature;

	// data for the humidity sensor
// 	int16_t currentHumidity;
//	int16_t currentTemperature;
//	uint32_t outputFromHumidityAndTemperature;
	// end of data for humidity sensor

_delay_ms(10000);

// write internalEEPROM to 0 to represent 0 bank of externalEEPROM
// TODO: THIS MAY NOT BE THE PLACE TO DO THIS, AS IF BATTERY RESET, THEN OVERWRITES DATA
	eeprom_write_byte( (uint8_t*) 1, 0);

/* -----------------------------------------
// Blink DISPLAY_LED on and off to show that it has started
	int i;
	for (i = 0; i<10; i++) {
		set_PORTB_bit(DISPLAY_LED,1);  // sets B0 as high
		_delay_ms(200);
		set_PORTB_bit(DISPLAY_LED,0); // sets B0 as low
		_delay_ms(200);
	}
// -------------------------------------------
*/

// write alarm to RTC for once per minute
//	_delay_ms(1000);			// If blinky light above remains, not required
//	set_RTC_bit(RTC_ADDR, 0x0E, 2, 1); //sets inct flag


  	set_RTC_bit(RTC_ADDR, 0x0E, 1, 1);		// set A1IE bit to 1, enable alarm

	// tells alarm to activate every minute on the 00 second mark
	set_RTC_bit(RTC_ADDR, 0x0D, 7, 1);	// set A2M4 bit to 1, 
	set_RTC_bit(RTC_ADDR, 0x0C, 7, 1);	// set A2M3 bit to 1
	set_RTC_bit(RTC_ADDR, 0x0B, 7, 1);	// set A2M2 bit to 1


// TURNED OFF ALARM SETTING 
/*
// set alarm 1
	set_RTC_bit(RTC_ADDR, 0x0E, 2, 1);		// set intcn bit to 1, enable alarm
	set_RTC_bit(RTC_ADDR, 0x0E, 0, 1);
	// tells alarm to activate every minute on the 00 second mark
	set_RTC_bit(RTC_ADDR, 0x08, 7, 1);	// set A2M4 bit to 1, 
	set_RTC_bit(RTC_ADDR, 0x09, 7, 1);	// set A2M3 bit to 1
	set_RTC_bit(RTC_ADDR, 0x0A, 7, 1);	// set A2M2 bit to 1
*/


// TODO: write initial time to beginning of externalEEPROM

// Begin while loop that will stop when externalEEPROM is full
//  EEPROM is full when both banks of 512kbits filled
	uint16_t basici;	// need to decide maximum number of measurements (128000 bytes/ info)
				// when 6 bytes of data, max points is 21333
				// when 8 bytes data, (sensors + hour + minute), max points is 16000
	uint16_t maxdata = 500; 	// magic number, be careful  


// ******************************************************************
// ------ BEGINNING OF LOOP THAT GOES AROUND AND AROUND -------------
// ******************************************************************

//	for (basici = 0; basici < (maxdata-8); basici++)
	for (;;)
	{ 	

// turn off interrupts
		cli();	
/*
  	USI_TWI_Master_Initialise();
	set_DDRB_bit(PAR_SENSOR,1);
	set_DDRB_bit(HUMIDITY,1);
	set_DDRB_bit(DISPLAY_LED,1);
*/
	set_DDRB_bit(PAR_SENSOR,1); 	// turn PAR_SENSOR into OUTPUT
	set_PORTB_bit(PAR_SENSOR,0);	// remove charge on capacitor on LED
	
	
	
// read time
	unsigned char temp;

  	uint8_t messageBuf[3];
	uint8_t timeBuf[3];
	messageBuf[0] = (RTC_ADDR << 1) | (0); 	// tell clock you are going to read it + W
	messageBuf[1] = 0x01; 			// write to clock where you want to read - minutes
	timeBuf[0] = (RTC_ADDR << 1) | (1);	// tell clock you are going to read it + R
	timeBuf[1] = 0xBE;
	timeBuf[2] = 0xEF;
	
	temp = USI_TWI_Start_Read_Write(messageBuf, 2);
	temp = USI_TWI_Start_Read_Write(timeBuf, 2);
	uint8_t minutes = timeBuf[1];		// saves minutes for later use
	uint8_t hours = 0;


	
// compare minute to determine if measurement needs to be taken

	int measurement_is_needed = 0;
	if   (HOW_OFTEN == 0) {measurement_is_needed = 1;}
	if ( (HOW_OFTEN == 1) && (minutes == 0) ) {measurement_is_needed = 1;}
	if ( (HOW_OFTEN == 2) && ( (minutes == 0) || (minutes == 0x30) ) ) {measurement_is_needed = 1;}
	if ( (HOW_OFTEN == 4) && ( (minutes == 0) || (minutes == 0x30) || (minutes == 0x15) || (minutes == 0x45) ) )
		{measurement_is_needed = 1;}
	if ( (HOW_OFTEN == 6) && ( (minutes==0) || (minutes==0x10) || (minutes==0x20) || (minutes==0x30) || (minutes==0x40) || (minutes==0x50) ) )
		{measurement_is_needed = 1;}



// ERROR CHECKING
//	measurement_is_needed = 1;
	_delay_ms(500);
	
// determine if a measurement is needed
	if (measurement_is_needed)
{
// Need to first collect hour measurements
// only done if measurement is needed!
	messageBuf[0] = (RTC_ADDR << 1) | (0); 	// tell clock you are going to read it + W
	messageBuf[1] = 0x02; 			// write to clock where you want to read - hours
	timeBuf[0] = (RTC_ADDR << 1) | (1);	// tell clock you are going to read it + R
	timeBuf[1] = 0xBE;
	timeBuf[2] = 0xEF;
	
	temp = USI_TWI_Start_Read_Write(messageBuf, 2);
	temp = USI_TWI_Start_Read_Write(timeBuf, 2);
	hours = timeBuf[1];
	
	
// Blink LED to show that it is taking a measurement
/* --------------------------------------------------------------
	_delay_ms(2000);
	for (i = 0; i<5; i++) {
		set_PORTB_bit(DISPLAY_LED,1);  // sets B0 as high
		_delay_ms(200);
		set_PORTB_bit(DISPLAY_LED,0); // sets B0 as low
		_delay_ms(200);
	}	
// ----------------------------------------------------------------
*/


// Take Light Measurements


/*
	// Turn on ADC
	ADCSRA |= (1<<ADEN);
	// set internal reference to 2.56V
	ADMUX |= (1<<REFS2);
	ADMUX |= (1<<REFS1);
	ADMUX &= ~(1<<REFS0);
	// set PAR_SENSOR as ADC
	ADMUX &= ~(1<<3); // clear MUX3
	ADMUX &= ~(1<<2); // clear MUX2
	ADMUX |= (PAR_SENSOR & 0b0010);
	ADMUX |= (PAR_SENSOR & 0b0001);
	
	_delay_ms(50);
	ADCSRA |= (1<<ADSC);
	while (ADCSRA & (1<<ADSC));
	_delay_ms(50);
	ADCSRA |= (1<<ADSC);
	while (ADCSRA & (1<<ADSC));
	
	uint16_t tempADCL = (uint16_t) ADCL;
	uint16_t tempADCH = (uint16_t) ADCH;
	
	// set flag to show we used 2.56V
	uint16_t PARdata = 0;
	PARdata = (tempADCH<<8) | (tempADCL);
	
	if (PARdata >= 440)
	{
		PARdata |= (0x8000);
	}
// if voltage is lower than 1.1V, we can change the reference to get a better measurement
	if (PARdata < 440)
	{
		ADMUX &= ~(1<<REFS2);
		_delay_ms(50);
		ADCSRA |= (1<<ADSC);
		while (ADCSRA & (1<<ADSC));
		_delay_ms(50);
		ADCSRA |= (1<<ADSC);
		while (ADCSRA & (1<<ADSC));
		
		tempADCL = (uint16_t) ADCL;
		tempADCH = (uint16_t) ADCH;
		PARdata = 0;
		PARdata = (tempADCH<<8) | (tempADCL);
	}
	
	// Turn off ADC
	ADCSRA &= ~(1<<ADEN);
	
*/	
	
// take light measurements - Capacitive Sensing
		uint16_t PARdata = readPAR_Sensor(PAR_SENSOR);

//		uint16_t PARdataADC = 0;
//		uint16_t PARdataADC = readPAR_SensorADC(PAR_SENSOR);

		uint16_t PARdataCAPADC = readPAR_SensorCAPADC(PAR_SENSOR);


//ERROR CHECKING		
//	set_DDRB_bit(PAR_SENSOR,1); 	// turn PAR_SENSOR into OUTPUT
//	set_PORTB_bit(PAR_SENSOR,0);	// remove charge on capacitor on LED

/* ---------------------------------------------------------------------
// take humidity measurement, repeat if needed
// Initialize Humidity sensor
	_delay_ms(2000);	// Humidity sensor needs 2 seconds to wake up
	uint32_t outputFromHumidityAndTemperature = 0;	
	int humiditycounter = 0;
	currentHumidity = 0;
	currentTemperature = 0;
	while (outputFromHumidityAndTemperature == 0)
	{
	// Measure humidity and temperature

// BLINK THE LED FOR HUMIDITY MEASUREMENTS
	for (i = 0; i<5; i++) {
		set_PORTB_bit(DISPLAY_LED,1);  // sets B0 as high
		_delay_ms(500);
		set_PORTB_bit(DISPLAY_LED,0); // sets B0 as low
		_delay_ms(500);
	}	

		outputFromHumidityAndTemperature = readHumidityAndTemperatureSensor(HUMIDITY);
		currentHumidity = 0x0000FFFF & outputFromHumidityAndTemperature;
		currentTemperature = (outputFromHumidityAndTemperature >> 16) & 0x0000FFFF;

		if (outputFromHumidityAndTemperature != 0) {break;}	// Humidity has been measured, get out of here
		// If we have tested humidity 4 times, humidity is broken and we need to go on with life
		currentHumidity = 0xEEFF;
		currentTemperature = 0xEEFF;
		if (humiditycounter == 5) {break;}
		humiditycounter++;	
		_delay_ms(2000);	// Humidity sensor needs 2 seconds between readings
	}
//----------------------------------------------------------------------
*/

int humiditytemperaturecheck = 0;
int humtempcounter = 0;



// data for the humidity sensor
	uint8_t retryCount;
	uint8_t bitTimes[DHT22_DATA_BIT_COUNT];

	uint8_t checkSum, csPart1, csPart2, csPart3, csPart4;
	int humidityerrorcode = 0;
// end of data for humidity sensor
// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
// BEGINNING OF HUMIDITY SENSOR
// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
while (humiditytemperaturecheck == 0)
{
	_delay_ms(2000);
	
		currentHumidity = 0;
 		currentTemperature = 0;
 		checkSum = 0;
		humidityerrorcode = 0;

// clear out all the bittimes
		for(i = 0; i < DHT22_DATA_BIT_COUNT; i++)
		{
    			bitTimes[i] = 0;
		}

// activate signal by dropping data line to LOW
// will have to change to use my functions
// and get rid of reg and bitmask
	cli();
	//CONVERT PIN TO OUTPUT
	// WRITE PIN AS LOW
  set_DDRB_bit(HUMIDITY,1);	//DIRECT_WRITE_LOW(reg, bitmask);
  set_PORTB_bit(HUMIDITY,0);	//DIRECT_MODE_OUTPUT(reg, bitmask); // Output Low
  sei();
  _delay_us(1100); // 1.1 ms

  cli();
  // CONVERT PIN TO INPUT
  // IN ORDER TO READ DATA
  set_DDRB_bit(HUMIDITY,0);//DIRECT_MODE_INPUT(reg, bitmask);	// Switch back to input so pin can float
  sei();
  
  // wait for sensor to respond
  // Find the start of the ACK Pulse
  retryCount = 0;
  do
  {
    if (retryCount > 25) //(Spec is 20 to 40 us, 25*2 == 50 us)
    {
      humidityerrorcode = 1; //DHT_ERROR_NOT_PRESENT;
	break;
    }
    retryCount++;
    _delay_us(2);
  } while(!(PINB & (1 << HUMIDITY)));
 // Find the end of the ACK Pulse
  retryCount = 0;
  do
  {
    if (retryCount > 50) //(Spec is 80 us, 50*2 == 100 us)
    {
      humidityerrorcode = 1;//DHT_ERROR_ACK_TOO_LONG;
	break;
    }
    retryCount++;
    _delay_us(2);
  } while((PINB & (1 << HUMIDITY)));

// Once all hands are shaken, begin transfering data from humidity sensor to mcu
 // Read the 40 bit data stream
  for(i = 0; i < DHT22_DATA_BIT_COUNT; i++)
  {
    // Find the start of the sync pulse
	// each bit starts with a 50us LOW followed by a 70us bit
    retryCount = 0;
    do
    {
      if (retryCount > 70) //(Spec is 50 us, 35*2 == 70 us)
      {
        humidityerrorcode = 1;//DHT_ERROR_SYNC_TIMEOUT;
	break;      
	}
      retryCount++;
      _delay_us(1);
    } while(!(PINB & (1 << HUMIDITY)));
    // Measure the width of the data pulse
    retryCount = 0;
    do
    {
      if (retryCount > 100) //(Spec is 80 us, 50*2 == 100 us)
      {
        humidityerrorcode = 1;//DHT_ERROR_DATA_TIMEOUT;
	break;      
	}
      retryCount++;
      _delay_us(1);
    } while((PINB & (1 << HUMIDITY)));
    bitTimes[i] = retryCount;
  }
  // Now bitTimes have the number of retries (us *2)
  // that were needed to find the end of each data bit
  // Spec: 0 is 26 to 28 us
  // Spec: 1 is 70 us
  // bitTimes[x] <= 11 is a 0
  // bitTimes[x] >  11 is a 1
  // Note: the bits are offset by one from the data sheet, not sure why
  for(i = 0; i < 16; i++)
  {
    if(bitTimes[i + 1] > HUMIDITY_CLOCK_TIME)
    {
      currentHumidity |= (1 << (15 - i));
	bitTimes[i+1] = 1;
    }	else { bitTimes[i+1] = 0; }
	
  }
  for(i = 0; i < 16; i++)
  {
    if(bitTimes[i + 17] > HUMIDITY_CLOCK_TIME)
    {
      currentTemperature |= (1 << (15 - i));
	bitTimes[i + 17] = 1;
    }	else { bitTimes[i+17] = 0; }
  }
  for(i = 0; i < 8; i++)
  {
    if(bitTimes[i + 33] > HUMIDITY_CLOCK_TIME)
    {
      checkSum |= (1 << (7 - i));
	bitTimes[i+33] = 1;
    }	else { bitTimes[i+33] = 0; }
  }
  
  // This is the checksum part for error checking
  // make sure the right values were read
  csPart1 = currentHumidity >> 8;
  csPart2 = currentHumidity & 0xFF;
  csPart3 = currentTemperature >> 8;
  csPart4 = currentTemperature & 0xFF;


// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!  
// DO I NEED TO CONVERT CURRENT HUMIDITY AS A FLOAT
// what about storing value as bit as converting to floats
// once data is downloaded?
// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
  /*_lastHumidity = (float(currentHumidity & 0x7FFF) / 10.0);
  if(currentTemperature & 0x8000)
  {
   // Below zero, non standard way of encoding negative numbers!
    currentTemperature &= 0x7FFF;
    _lastTemperature = (float(currentTemperature) / 10.0) * -1.0;
  }
  else
  {
    _lastTemperature = float(currentTemperature) / 10.0;
  }
  */
  currentHumidity &= 0x7FFF; // int that needs to be divided by 10.0 to get %
  if (currentTemperature & 0x8000) 
  {
	// temperature is below zero and stored strange
	currentTemperature &= 0x7FFF;
	currentTemperature *= -1; 	// int that needs to be divided by 10 to get temperature
	} 
	// Temperature is positive value and do nothing
	
// do checksum on unmanipulated values
  if (!(checkSum == ((csPart1 + csPart2 + csPart3 + csPart4) & 0xFF)))
  {
    humidityerrorcode = 1;//DHT_ERROR_CHECKSUM;
  }
	
	
	humiditytemperaturecheck = 1;
	if (humidityerrorcode)
	{
	currentTemperature = 0;
	currentHumidity = 0;
	humiditytemperaturecheck = 0;
	}

// determine if we need to check humidity again 
// or if we have tried too many times
	
	humtempcounter++;
	if (humtempcounter >= 5) {break;}
	if (humiditytemperaturecheck == 1) {break;}
	
// --------------------------------------------------------------------
// --------------------------------------------------------------------

} // end while loop (humiditytemperaturecheck == 0)

	
// TODO: write to EEPROM

		uint8_t messageBuf[MESSAGEBUF_SIZE];
//		unsigned char temp;
	// THIS CODE IS FOR 32k EEPROM
	// TODO: UPDATE TO 1024k EEPROM

// TODO: ADD IN SOME TIME COMPONENT
		messageBuf[0] = (MEMORY_ADDR<<TWI_ADR_BITS) | (FALSE<<TWI_READ_BIT);
		messageBuf[1] = (externaleepromcounter>>8) & 0b00001111;       	// location to be written 4 high bits
		messageBuf[2] = (externaleepromcounter & 0b11111111);		// location to be written, 8 low bits
		messageBuf[3] = minutes;					// MAYBE PUT HOURS HERE
		messageBuf[4] = hours;					// write minutes
		externaleepromcounter += 2;
		temp = USI_TWI_Start_Read_Write( messageBuf, 5);
		_delay_ms(50);


		messageBuf[0] = (MEMORY_ADDR<<TWI_ADR_BITS) | (FALSE<<TWI_READ_BIT);
		messageBuf[1] = (externaleepromcounter>>8) & 0b00001111;       	// location to be written 4 high bits
		messageBuf[2] = (externaleepromcounter & 0b11111111);		// location to be written, 8 low bits
		messageBuf[3] = (PARdata>>8) & 0b11111111;			// 8 bits of data high PAR
		messageBuf[4] = (PARdata & 0b11111111);				// 8 bits of data low PAR
		externaleepromcounter += 2;
		temp = USI_TWI_Start_Read_Write( messageBuf, 5);
		_delay_ms(50);
	/*	
		messageBuf[0] = (MEMORY_ADDR<<TWI_ADR_BITS) | (FALSE<<TWI_READ_BIT);
		messageBuf[1] = (externaleepromcounter>>8) & 0b00001111;       	// location to be written 4 high bits
		messageBuf[2] = (externaleepromcounter & 0b11111111);		// location to be written, 8 low bits
		messageBuf[3] = (PARdataADC>>8) & 0b11111111;			// 8 bits of data high PAR
		messageBuf[4] = (PARdataADC & 0b11111111);				// 8 bits of data low PAR
		externaleepromcounter += 2;
		temp = USI_TWI_Start_Read_Write( messageBuf, 5);
		_delay_ms(50);
	*/	
		messageBuf[0] = (MEMORY_ADDR<<TWI_ADR_BITS) | (FALSE<<TWI_READ_BIT);
		messageBuf[1] = (externaleepromcounter>>8) & 0b00001111;       	// location to be written 4 high bits
		messageBuf[2] = (externaleepromcounter & 0b11111111);		// location to be written, 8 low bits
		messageBuf[3] = (PARdataCAPADC>>8) & 0b11111111;			// 8 bits of data high PAR
		messageBuf[4] = (PARdataCAPADC & 0b11111111);				// 8 bits of data low PAR
		externaleepromcounter += 2;
		temp = USI_TWI_Start_Read_Write( messageBuf, 5);
		_delay_ms(50);

		messageBuf[0] = (MEMORY_ADDR<<TWI_ADR_BITS) | (FALSE<<TWI_READ_BIT);
		messageBuf[1] = (externaleepromcounter>>8) & 0b00001111;       	// location to be written 4 high bits
		messageBuf[2] = (externaleepromcounter & 0b11111111);		// location to be written, 8 low bits
		messageBuf[3] = (currentTemperature>>8) & 0b11111111;		// 8 bits high TEMP
		messageBuf[4] = (currentTemperature & 0b11111111);		// 8 bits low TEMP
		externaleepromcounter += 2;
		temp = USI_TWI_Start_Read_Write( messageBuf, 5);
		_delay_ms(50);

		messageBuf[0] = (MEMORY_ADDR<<TWI_ADR_BITS) | (FALSE<<TWI_READ_BIT);
		messageBuf[1] = (externaleepromcounter>>8) & 0b00001111;       	// location to be written 4 high bits
		messageBuf[2] = (externaleepromcounter & 0b11111111);		// location to be written, 8 low bits
		messageBuf[3] = (currentHumidity>>8) & 0b11111111;		// 8 bits low HUMIDITY
		messageBuf[4] = (currentHumidity & 0b11111111);			// 8 bits low HUMIDITY
		externaleepromcounter += 2;
		temp = USI_TWI_Start_Read_Write( messageBuf, 5);
		_delay_ms(50);

} // end of if (MEASUREMENT IS NEEDED)

// turn off alarm on RTC
	set_RTC_bit(RTC_ADDR, 0x0F, 1, 0);	// turn off A1F alarm


// put to sleep

	sleep_enable();
//	sleep_bod_disable();		// TODO: FIGURE OUT .h FILE
	sei();
	sleep_cpu();
	sleep_disable();		

//	_delay_ms(5000);

	} // end for (basici=0; basici < (maxdata-6); basici++)

	return 0;
} // end int main (void)


// -------- BEGIN FUNCTIONS ----------------------------------------------------

ISR(PCINT0_vect) 
{
};

int set_RTC_bit (uint8_t device, uint8_t address, uint8_t bit, uint8_t value)
{
	uint8_t clockBuf[3];
	uint8_t tempBuf[3];
	unsigned char temp;			// var used in all read write situations to hold error throws

	// First need to tell RTC what address I want to read
	// then I need to request data in address from RTC
	// I then take data, add flag, and send it back
	clockBuf[0] = (device << 1) | (0); 	// tell clock you are going to talk to it + W
	clockBuf[1] = address; 			// tell clock you want to read alarm bit
	tempBuf[0] = (device << 1) | (1);
	tempBuf[1] = 0xFF;
	 
	temp = USI_TWI_Start_Read_Write(clockBuf, 2);
	temp = USI_TWI_Start_Read_Write(tempBuf, 2);


	//clockBuf[2] = clockBuf[1] | (1<<1);	//
	if (value == 0) {tempBuf[1] &= ~(1<<bit);}
	if (value == 1) {tempBuf[1] |= (1<<bit);}
	clockBuf[2] = tempBuf[1];

	temp = USI_TWI_Start_Read_Write(clockBuf, 3);
//	_delay_ms(10);

	return 1;
}

uint16_t readPAR_SensorADC (int pinPAR_SENSOR)
{
		int i;
	
// For PAR reading, we are going to take ten quick readings and then average
// this is based of capacitance sensing
		uint32_t PARaverage = 0;
		uint16_t returnPARdata = 0;		

	
		int j;
		
	// Turn on ADC
	ADCSRA |= (1<<ADEN);
	// set internal reference to 2.56V
	ADMUX |= (1<<REFS2);
	ADMUX |= (1<<REFS1);
	ADMUX &= ~(1<<REFS0);
	// get ready to set PAR_SENSOR as ADC
	ADMUX &= ~(1<<3); // clear MUX3
	ADMUX &= ~(1<<2); // clear MUX2
	// Change pin to ADC
	ADMUX |= (PAR_SENSOR & 0b0010);
	ADMUX |= (PAR_SENSOR & 0b0001);
	
	uint16_t tempADCL = 0;//(uint16_t) ADCL;
	uint16_t tempADCH = 0;//(uint16_t) ADCH;
	uint16_t PARdata = 0;
	uint16_t PARdatafinal= 0;
	
	
		for (j =0; j<10; j++) {
			// first remove residual charge on LED
			set_DDRB_bit(pinPAR_SENSOR,1); 	// turn PAR_SENSOR into OUTPUT
			set_PORTB_bit(pinPAR_SENSOR,0);	// remove charge on capacitor on LED
			_delay_ms(100);
			// charge up LED capacitor
			set_PORTB_bit(pinPAR_SENSOR,1);
			_delay_ms(100);	
			// set negPB3 to input
			set_DDRB_bit(pinPAR_SENSOR,0);
			// drop negPB3 to LOW		
			set_PORTB_bit(pinPAR_SENSOR,0);	
			
			// Turn on ADC
			ADCSRA |= (1<<ADEN);
			
			// Collect a measurement and throw out first measurement
				ADCSRA |= (1<<ADSC);
				while (ADCSRA & (1<<ADSC));
	//			tempADCL = (uint16_t) ADCL;
//				tempADCH = (uint16_t) ADCH;
				PARdata = (ADCL) | (ADCH<<8);
				

			
// First we wait until the LED measures right below 2.56V
// we then wait 5 ms and take another ADC measurement
// Then we record the difference between the two values
			// take another measurement
			ADCSRA |= (1<<ADSC);
			while (ADCSRA & (1<<ADSC));
			PARdata = (ADCL) | (ADCH<<8);
			while (PARdata > 1022)
			{
				ADCSRA |= (1<<ADSC);
				while (ADCSRA & (1<<ADSC));
				PARdata = (ADCL) | (ADCH<<8);
			}
			_delay_ms(5);
			ADCSRA |= (1<<ADSC);
			while (ADCSRA & (1<<ADSC));
			PARdatafinal = (ADCL) | (ADCH<<8);
			uint16_t tempPARdata = PARdata - PARdatafinal;
			PARaverage += (uint32_t) tempPARdata;
			
					// Turn off ADC
			ADCSRA &= ~(1<<ADEN);
			_delay_ms(100);
		} // end int i for loop
		
		PARaverage = (PARaverage/10);
		returnPARdata = (uint16_t) PARaverage;
		

		
		return(returnPARdata);
}


uint16_t readPAR_Sensor (int pinPAR_SENSOR)
{
// For PAR reading, we are going to take ten quick readings and then average
// this is based of capacitance sensing
		uint32_t PARaverage = 0;
		uint16_t returnPARdata = 0;		

		int i;
		int j;
		for (i =0; i<10; i++) {
			// first remove residual charge on LED
			set_DDRB_bit(pinPAR_SENSOR,1); 	// turn PAR_SENSOR into OUTPUT
			set_PORTB_bit(pinPAR_SENSOR,0);	// remove charge on capacitor on LED
			_delay_ms(100);
			// charge up LED capacitor
			set_PORTB_bit(pinPAR_SENSOR,1);
			_delay_ms(100);		

			// set negPB3 to input
			set_DDRB_bit(pinPAR_SENSOR,0);
			// drop negPB3 to LOW		
			set_PORTB_bit(pinPAR_SENSOR,0);		
		
			for (j=0; j<30000; j++) {
				if ((PINB & (1 << pinPAR_SENSOR) )==0) {break;}
			}
			PARaverage += (uint32_t) j;
			_delay_ms(100);
		} // end int i for loop
		
		PARaverage = (PARaverage/10);
		returnPARdata = (uint16_t) PARaverage;
		return(returnPARdata);
}



int set_PORTB_bit (int position, int value) 
{
	// taken from imakeprojects.com/Projects/avr-tutorial
	
	if (value ==0) { PORTB &= ~(1 << position); }
	else { PORTB |= (1 << position);}

	return 1;
}

int set_DDRB_bit (int position, int value)
{
	if (value==0) { DDRB &= ~(1<<position); }
	else { DDRB |= (1<<position); }
	return 1;
}

uint16_t readPAR_SensorCAPADC (int pinPAR_SENSOR)
{
		int i;
	
// For PAR reading, we are going to take ten quick readings and then average
// this is based of capacitance sensing
		uint32_t PARaverage = 0;
		uint16_t returnPARdata = 0;		

	
		int j;
		
	// Turn on ADC
	ADCSRA |= (1<<ADEN);
	// set internal reference to 2.56V
	ADMUX |= (1<<REFS2);
	ADMUX |= (1<<REFS1);
	ADMUX &= ~(1<<REFS0);
	// get ready to set PAR_SENSOR as ADC
	ADMUX &= ~(1<<3); // clear MUX3
	ADMUX &= ~(1<<2); // clear MUX2
	// Change pin to ADC
	ADMUX |= (PAR_SENSOR & 0b0010);
	ADMUX |= (PAR_SENSOR & 0b0001);
	
	uint16_t tempADCL = 0;//(uint16_t) ADCL;
	uint16_t tempADCH = 0;//(uint16_t) ADCH;
	uint16_t PARdata = 0;
	uint16_t PARdatafinal= 0;
	
	
		for (i =0; i<10; i++) {
			// first remove residual charge on LED
			set_DDRB_bit(pinPAR_SENSOR,1); 	// turn PAR_SENSOR into OUTPUT
			set_PORTB_bit(pinPAR_SENSOR,0);	// remove charge on capacitor on LED
			_delay_ms(100);
			// charge up LED capacitor
			set_PORTB_bit(pinPAR_SENSOR,1);
			_delay_ms(100);	
			// set negPB3 to input
			set_DDRB_bit(pinPAR_SENSOR,0);
			// drop negPB3 to LOW		
			set_PORTB_bit(pinPAR_SENSOR,0);	
			
			// Turn on ADC
			ADCSRA |= (1<<ADEN);
			
			// Collect a measurement and throw out first measurement
				ADCSRA |= (1<<ADSC);
				while (ADCSRA & (1<<ADSC));
	//			tempADCL = (uint16_t) ADCL;
//				tempADCH = (uint16_t) ADCH;
				PARdata = (ADCL) | (ADCH<<8);
					
			// we wait until ADC reads just below 2.56V then we switch
			// back to capacitance measuring and time how long before we
			// get digital zero


			// take another measurement
			ADCSRA |= (1<<ADSC);
			while (ADCSRA & (1<<ADSC));
			PARdata = (ADCL) | (ADCH<<8);
			while (PARdata > 1022)
			{
				ADCSRA |= (1<<ADSC);
				while (ADCSRA & (1<<ADSC));
				PARdata = (ADCL) | (ADCH<<8);
			}
			
			// now we measure until pin is digital zero
			// Turn off ADC
			ADCSRA &= ~(1<<ADEN);
			
			for (j=0; j<30000; j++) {
				if ((PINB & (1 << pinPAR_SENSOR) )==0) {break;}
			}
			PARaverage += (uint32_t) j;
		
					
			_delay_ms(100);
		} // end int i for loop
		
		PARaverage = (PARaverage/10);
		returnPARdata = (uint16_t) PARaverage;
		return(returnPARdata);
}
