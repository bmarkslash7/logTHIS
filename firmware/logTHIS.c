/*********************************************
ver2 of datalogger:
Wake up from alarm from RTC once a minute
Determine whether uC needs to collect data
Read from Humidity sensor (keep trying a couple of times if initial fails
Read from Light sensor - convert cap sensor to ADC
Write data to EEPROM
Put AVR to sleep
 * Author: christopher stieha and abram connelly
 * Filename: logTHIS.c
 * Chip: ATtiny85

I2c code from http://www.instructables.com/id/I2C_Bus_for_ATtiny_and_ATmega/ user doctek
HUMIDITY code modified from https://github.com/nethoncho/Arduino-DHT22 user nethoncho
 */


/* logTHIS will look at the header in bank 0 of the EEPROM to determine
 * the logging policy (wraparound, stop at full, etc.), the frequency (1 minute,
 * 10 minute, 15 minute etc.) and the number of address pointers once at 
 * bootup.  
 *
 * It will then write an information record to the EEPROM with the policy, frequency and 
 * it's version number.
 *
 * It will then write an information record for each of the temperature, humidity
 * and PAR to the EEPROM.
 *
 * It will get the date from the RTC and then write a datestamp record to EEPROM.
 *
 * After meta information has been read, it will then look at the logging address
 * pointers to pick the latest (*) logging pointer position.
 *
 * At each each logging event, it will write a loggin record to the EEPROM.  It will then
 * write update the next logging address pointer in the logging address pointer list.
 *
 * The latest logging address pointer is taken with the understanding that wraparound
 * can occur.
 *
 *
 *
 * header format:
 *
 *        policy : frequency |   version   |   version   |   reserved  
 *      -----------------------------------------------------------------
 * bits |   4         4      |     8       |      8      |     5*8 = 40 |
 *      -----------------------------------------------------------------
 *
 *      total bytes: 8
 *
 * ***********************************
 *
 * log address pointer format:
 *       
 *        sequence | bank | address high | address low
 *      -----------------------------------------------
 * bits |    8     |  8   |      8       |     8      |
 *      -----------------------------------------------
 *
 *      total bytes: 4
 *
 * ***********************************
 * 
 * record format:
 *
 *         record type : record length |  data
 *      -----------------------------------------
 * bits |       4              4       |  0-16  |
 *      -----------------------------------------
 *
 *      total bytes: variable (1 to 17)
 *
 *      record length signifies the number of bytes of data in the record
 *
 * record type  | record value  | description
 * -------------------------------------------
 *  Information |      0        | holds header informat or which field
 *              |               | in a data record belongs to which sensor
 * -------------------------------------------
 *  Datestamp   |      1        | records date from RTC
 * -------------------------------------------
 *  Data        |      2        | holds data from sensors
 * -------------------------------------------
 *  Event       |      3        | holds miscellaneous event information (unused for now)
 * -------------------------------------------
 *
 */


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
#define HUMIDITY_CLOCK_TIME 6       // number compared to timing of humidity sensors to determine 0 or 1

// EEPROM definitions
#define EEPROM_I2C_ADDR0  0b1010000    // 1M, first bank EEPROM
#define EEPROM_I2C_ADDR1  0b1011000    // 1M, second bank EEPROM

#define EEPROM_BANK_SIZE 62500          // in bytes

#define I2C_DELAY_MS 50

// RTC definitions
#define RTC_I2C_ADDR 0b1101000    // function needs to <<1 and add R/W bit


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
void get_rtc_str_date(uint8_t *dt);

void setup_boot_information(void);
void setup_xeprom_address_vars(void);
void setup_default(void);
void update_xeprom_address_pointers(void);
uint8_t update_xeprom_counters(uint8_t chunk);

void write_eeprom(uint16_t);
void write_xeprom_2(uint8_t, uint8_t);
void write_xeprom_1(uint8_t byt);
void read_eeprom(uint8_t *, uint8_t );

void write_record( uint8_t record_type, uint8_t record_len, uint8_t *buf);

#define HEADER_SIZE 8
#define ADDRESS_POINTER_SIZE 4

#define RECORD_INFO  ( 0 << 4 )
#define RECORD_DATE  ( 1 << 4 )
#define RECORD_DATA  ( 2 << 4 )
#define RECORD_EVENT ( 3 << 4 )

enum{
  LOG_FREQ_1M = 0,
  LOG_FREQ_10M ,
  LOG_FREQ_15M ,
  LOG_FREQ_30M ,
  LOG_FREQ_1H 
};


enum {
  EEPROM_POLICY_STOP = 0,
  EEPROM_POLICY_WRAP
};

typedef struct logTHIS_state_type
{
  uint8_t header[HEADER_SIZE];
  uint8_t eeprom_policy;
  uint8_t log_frequency;

  uint8_t n_address_pointer;
  uint8_t address_pointer_seq;
  uint8_t address_pointer_pos;
  uint8_t eeprom_i2c_address;
  uint16_t eeprom_mem_address;
  uint8_t eeprom_full;

  uint8_t default_init;

} logTHIS_state_t;

logTHIS_state_t state = { {0} };


// defaults
#define DEFAULT_N_ADDRESS               8
//#define DEFAULT_FREQ                    LOG_FREQ_30M
#define DEFAULT_FREQ                    LOG_FREQ_1M
#define DEFAULT_EEPROM                  EEPROM_I2C_ADDR0
#define DEFAULT_EEPROM_POLICY           EEPROM_POLICY_STOP


// temporary, should just go to sleep and never wake up
// if we've gotten to a log full (without wraparound) state.
//
void log_full_state(void)
{
  set_DDRB_bit(1,1);
  for (;;)
  {
    set_PORTB_bit(1,1);  // sets B0 as high
    _delay_ms(100);
    set_PORTB_bit(1,0); // sets B0 as low
    _delay_ms(100);
  }
  set_DDRB_bit(1,0);
}

void blink_always(void)
{
  set_DDRB_bit(1,1);
  for (;;)
  {
    set_PORTB_bit(1,1);  // sets B0 as high
    _delay_ms(100);
    set_PORTB_bit(1,0); // sets B0 as low
    _delay_ms(100);
  }
  set_DDRB_bit(1,0);
}


// ------------------HERE STARTS MAIN ----------------------------------------------
int main (void)
{
    // initialize variables
    int i;
    int16_t currentHumidity;
    int16_t currentTemperature;

    uint8_t rtc_str_date[14];

    // configure sleep state and interrupt pin
    set_sleep_mode(SLEEP_MODE_PWR_DOWN);

    // -------------------------------------------

    GIMSK |= (1 << 5);  // enable interrupts
    PCMSK |= (1 << 1);  // set Pin6/PB1/PCINT1 as interupt pin


    // initialize pins
    USI_TWI_Master_Initialise();
    set_DDRB_bit(PAR_SENSOR,1);
    set_DDRB_bit(HUMIDITY,1);

    //_delay_ms(10000);


    //************* DEBUG *****************
    //  setup_default();
    //************* DEBUG *****************

    // write internalEEPROM to 0 to represent 0 bank of externalEEPROM
    // TODO: THIS MAY NOT BE THE PLACE TO DO THIS, AS IF BATTERY RESET, THEN OVERWRITES DATA
    //eeprom_write_byte( (uint8_t*) 1, 0);

    //-------------------------------
    // SETUP on boot
    //
    state.eeprom_i2c_address = EEPROM_I2C_ADDR0;
    state.eeprom_full = 0;
    state.default_init = 0;
    setup_boot_information();
    setup_xeprom_address_vars();

    // in case we have factory eeprom
    // store some default values, re-read
    if (state.default_init)
    {
      setup_default();
      setup_boot_information();
      setup_xeprom_address_vars();
    }


    // write header event
    write_record( RECORD_EVENT, 4, (uint8_t *)"boot");
    write_record( RECORD_INFO, 5, (uint8_t *)("\0" "Temp") );
    write_record( RECORD_INFO, 4, (uint8_t *)("\1" "Hum") );
    write_record( RECORD_INFO, 4, (uint8_t *)("\2" "PAR") );


    // write header event
    update_xeprom_counters(6);
    if (state.eeprom_full) log_full_state();
    write_xeprom_1( RECORD_INFO | 5 );
    write_xeprom_1( (state.eeprom_policy << 4) | (state.log_frequency) );
    write_xeprom_1( state.n_address_pointer );
    write_xeprom_1( state.eeprom_i2c_address );
    write_eeprom( state.eeprom_mem_address );

    // read RTC time and write date record
    // placeholder for now
    get_rtc_str_date(rtc_str_date);
    write_record( RECORD_DATE, 14, rtc_str_date);
    update_xeprom_address_pointers();


    // *************************************************
    // DEBUG!!
    //log_full_state();
    //blink_always();

    /*
    sleep_enable();
    sei();
    sleep_cpu();
    sleep_disable();
    blink_always();
    */

    // *************************************************

    //-------------------------------


    // write alarm to RTC for once per minute
    set_RTC_bit(RTC_I2C_ADDR, 0x0E, 1, 1);    // set A1IE bit to 1, enable alarm

    // tells alarm to activate every minute on the 00 second mark
    set_RTC_bit(RTC_I2C_ADDR, 0x0D, 7, 1);  // set A2M4 bit to 1,
    set_RTC_bit(RTC_I2C_ADDR, 0x0C, 7, 1);  // set A2M3 bit to 1
    set_RTC_bit(RTC_I2C_ADDR, 0x0B, 7, 1);  // set A2M2 bit to 1

    // ******************************************************************
    // ------ BEGINNING OF LOOP THAT GOES AROUND AND AROUND -------------
    // ******************************************************************

    for (;;)
    {

        cli();

        set_DDRB_bit(PAR_SENSOR,1);   // turn PAR_SENSOR into OUTPUT
        set_PORTB_bit(PAR_SENSOR,0);  // remove charge on capacitor on LED

        // read time
        unsigned char temp;

        uint8_t messageBuf[3];
        uint8_t timeBuf[3];
        messageBuf[0] = (RTC_I2C_ADDR << 1) | (0);   // tell clock you are going to read it + W
        messageBuf[1] = 0x01;       // write to clock where you want to read - minutes
        timeBuf[0] = (RTC_I2C_ADDR << 1) | (1);  // tell clock you are going to read it + R
        timeBuf[1] = 0xBE;
        timeBuf[2] = 0xEF;

        temp = USI_TWI_Start_Read_Write(messageBuf, 2);
        _delay_ms(I2C_DELAY_MS);

        temp = USI_TWI_Start_Read_Write(timeBuf, 2);
        _delay_ms(I2C_DELAY_MS);

        uint8_t minutes = timeBuf[1];    // saves minutes for later use
        uint8_t hours = 0;



        // compare minute to determine if measurement needs to be taken
        // minute time in BCD
        int measurement_is_needed = 0;
        if   (state.log_frequency == LOG_FREQ_1M )
            measurement_is_needed = 1;
        if ( (state.log_frequency == LOG_FREQ_10M ) &&
                ( (minutes==0) ||
                  (minutes==0x10) ||
                  (minutes==0x20) ||
                  (minutes==0x30) ||
                  (minutes==0x40) ||
                  (minutes==0x50) ) )
            measurement_is_needed = 1;
        if ( (state.log_frequency == LOG_FREQ_15M ) &&
                ( (minutes == 0) ||
                  (minutes == 0x15) ||
                  (minutes == 0x30) ||
                  (minutes == 0x45) ) )
            measurement_is_needed = 1;
        if ( (state.log_frequency == LOG_FREQ_30M ) &&
                ( (minutes == 0) ||
                  (minutes == 0x30) ) )
            measurement_is_needed = 1;
        if ( (state.log_frequency == LOG_FREQ_1H ) &&
                (minutes == 0) )
            measurement_is_needed = 1;



        _delay_ms(500);

        // determine if a measurement is needed
        if (measurement_is_needed)
        {
            // Need to first collect hour measurements
            // only done if measurement is needed!
            messageBuf[0] = (RTC_I2C_ADDR << 1) | (0);   // tell clock you are going to read it + W
            messageBuf[1] = 0x02;       // write to clock where you want to read - hours
            timeBuf[0] = (RTC_I2C_ADDR << 1) | (1);  // tell clock you are going to read it + R
            timeBuf[1] = 0xBE;
            timeBuf[2] = 0xEF;

            temp = USI_TWI_Start_Read_Write(messageBuf, 2);
            _delay_ms(I2C_DELAY_MS);

            temp = USI_TWI_Start_Read_Write(timeBuf, 2);
            _delay_ms(I2C_DELAY_MS);

            hours = timeBuf[1];

            // take light measurements - Capacitive Sensing
            uint16_t PARdata = readPAR_Sensor(PAR_SENSOR);
            uint16_t PARdataCAPADC = readPAR_SensorCAPADC(PAR_SENSOR);

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
                    bitTimes[i] = 0;

                // activate signal by dropping data line to LOW
                // will have to change to use my functions
                // and get rid of reg and bitmask
                cli();
                set_DDRB_bit(HUMIDITY,1);   //CONVERT PIN TO OUTPUT
                set_PORTB_bit(HUMIDITY,0);  // WRITE PIN AS LOW
                sei();

                _delay_us(1100); // 1.1 ms

                cli();
                set_DDRB_bit(HUMIDITY,0); // Switch back to input so pin can float
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
                }
                while((PINB & (1 << HUMIDITY)));

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
                    }  else {
                        bitTimes[i+1] = 0;
                    }

                }
                for(i = 0; i < 16; i++)
                {
                    if(bitTimes[i + 17] > HUMIDITY_CLOCK_TIME)
                    {
                        currentTemperature |= (1 << (15 - i));
                        bitTimes[i + 17] = 1;
                    }  else {
                        bitTimes[i+17] = 0;
                    }
                }
                for(i = 0; i < 8; i++)
                {
                    if(bitTimes[i + 33] > HUMIDITY_CLOCK_TIME)
                    {
                        checkSum |= (1 << (7 - i));
                        bitTimes[i+33] = 1;
                    }  else {
                        bitTimes[i+33] = 0;
                    }
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
                currentHumidity &= 0x7FFF; // int that needs to be divided by 10.0 to get %
                if (currentTemperature & 0x8000)
                {
                    // temperature is below zero and stored strange
                    currentTemperature &= 0x7FFF;
                    currentTemperature *= -1;   // int that needs to be divided by 10 to get temperature
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
                if (humtempcounter >= 5) {
                    break;
                }
                if (humiditytemperaturecheck == 1) {
                    break;
                }

              // --------------------------------------------------------------------
              // --------------------------------------------------------------------

            } // end while loop (humiditytemperaturecheck == 0)


            // if we've switched banks, write relevant information
            // recoards as first entries
            if ( update_xeprom_counters(5) && 
                 (state.eeprom_policy == EEPROM_POLICY_WRAP) )
            {
              write_record( RECORD_INFO, 5, (uint8_t *)("\0" "Temp") );
              write_record( RECORD_INFO, 4, (uint8_t *)("\1" "Hum") );
              write_record( RECORD_INFO, 4, (uint8_t *)("\2" "PAR") );
                
              get_rtc_str_date(rtc_str_date);
              write_record( RECORD_DATE, 14, rtc_str_date);
              update_xeprom_address_pointers();
            }

            if (!state.eeprom_full)
              write_xeprom_1( RECORD_DATA | 4 );

            if (!state.eeprom_full)
              write_eeprom(currentTemperature);

            if (!state.eeprom_full)
              write_eeprom(currentHumidity);

            if (!state.eeprom_full)
              update_xeprom_address_pointers();

        } // end of if (MEASUREMENT IS NEEDED)

        // turn off alarm on RTC
        set_RTC_bit(RTC_I2C_ADDR, 0x0F, 1, 0);  // turn off A1F alarm


        // put to sleep
        sleep_enable();
        sei();
        sleep_cpu();
        sleep_disable();


    }

    return 0;
} 




ISR(PCINT0_vect)
{
};

int set_RTC_bit (uint8_t device, uint8_t address, uint8_t bit, uint8_t value)
{
    uint8_t clockBuf[3];
    uint8_t tempBuf[3];
    unsigned char temp;      // var used in all read write situations to hold error throws

    // First need to tell RTC what address I want to read
    // then I need to request data in address from RTC
    // I then take data, add flag, and send it back
    clockBuf[0] = (device << 1) | (0);   // tell clock you are going to talk to it + W
    clockBuf[1] = address;       // tell clock you want to read alarm bit
    tempBuf[0] = (device << 1) | (1);
    tempBuf[1] = 0xFF;

    temp = USI_TWI_Start_Read_Write(clockBuf, 2);
    _delay_ms(I2C_DELAY_MS);

    temp = USI_TWI_Start_Read_Write(tempBuf, 2);
    _delay_ms(I2C_DELAY_MS);

    if (value == 0) {
        tempBuf[1] &= ~(1<<bit);
    }
    if (value == 1) {
        tempBuf[1] |= (1<<bit);
    }
    clockBuf[2] = tempBuf[1];

    temp = USI_TWI_Start_Read_Write(clockBuf, 3);
    _delay_ms(I2C_DELAY_MS);

    return 1;
}

uint16_t readPAR_SensorADC (int pinPAR_SENSOR)
{

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

    uint16_t PARdata = 0;
    uint16_t PARdatafinal= 0;


    for (j =0; j<10; j++) 
    {

        // first remove residual charge on LED
        set_DDRB_bit(pinPAR_SENSOR,1);   // turn PAR_SENSOR into OUTPUT
        set_PORTB_bit(pinPAR_SENSOR,0);  // remove charge on capacitor on LED
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
        _delay_ms(I2C_DELAY_MS);

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
    int i, j;

    for (i =0; i<10; i++) 
    {
        // first remove residual charge on LED
        set_DDRB_bit(pinPAR_SENSOR,1);   // turn PAR_SENSOR into OUTPUT
        set_PORTB_bit(pinPAR_SENSOR,0);  // remove charge on capacitor on LED
        _delay_ms(100);
        // charge up LED capacitor
        set_PORTB_bit(pinPAR_SENSOR,1);
        _delay_ms(100);

        // set negPB3 to input
        set_DDRB_bit(pinPAR_SENSOR,0);
        // drop negPB3 to LOW
        set_PORTB_bit(pinPAR_SENSOR,0);

        for (j=0; j<30000; j++) 
        {
            if ((PINB & (1 << pinPAR_SENSOR) )==0) 
                break;
        }
        PARaverage += (uint32_t) j;
        _delay_ms(100);
    } // end int i for loop

    PARaverage = (PARaverage/10);
    returnPARdata = (uint16_t) PARaverage;
    return(returnPARdata);
}



// taken from imakeprojects.com/Projects/avr-tutorial
int set_PORTB_bit (int position, int value)
{
    if (value ==0) 
        PORTB &= ~(1 << position);
    else 
        PORTB |= (1 << position);

    return 1;
}

int set_DDRB_bit (int position, int value)
{
    if (value==0) 
        DDRB &= ~(1<<position);
    else 
        DDRB |= (1<<position);
    return 1;
}

uint16_t readPAR_SensorCAPADC (int pinPAR_SENSOR)
{

    // For PAR reading, we are going to take ten quick readings and then average
    // this is based of capacitance sensing
    uint32_t PARaverage = 0;
    uint16_t returnPARdata = 0;
    int i, j;

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

    uint16_t PARdata = 0;

    for (i =0; i<10; i++) 
    {

        // first remove residual charge on LED
        set_DDRB_bit(pinPAR_SENSOR,1);   // turn PAR_SENSOR into OUTPUT
        set_PORTB_bit(pinPAR_SENSOR,0);  // remove charge on capacitor on LED
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

        for (j=0; j<30000; j++) 
            if ((PINB & (1 << pinPAR_SENSOR) )==0) 
                break;
        PARaverage += (uint32_t) j;


        _delay_ms(100);
    } // end int i for loop

    PARaverage = (PARaverage/10);
    returnPARdata = (uint16_t) PARaverage;
    return(returnPARdata);
}

uint8_t update_xeprom_counters(uint8_t chunk) 
{
  uint8_t switched_bank=0;

  if ( (state.eeprom_mem_address + chunk) > EEPROM_BANK_SIZE )
  {

    // wrap around
    if      ( (state.eeprom_policy == EEPROM_POLICY_WRAP) &&
              (state.eeprom_i2c_address == EEPROM_I2C_ADDR1) )
    {
      switched_bank = 1;
      state.eeprom_i2c_address = EEPROM_I2C_ADDR0;
      state.eeprom_mem_address = HEADER_SIZE + (ADDRESS_POINTER_SIZE * state.n_address_pointer);
    }

    // signal stop
    else if ( (state.eeprom_policy == EEPROM_POLICY_STOP) &&
              (state.eeprom_i2c_address == EEPROM_I2C_ADDR1) )
    {
      state.eeprom_full = 1;
    }

    // otherwise advance to next bank
    else
    {
      switched_bank = 1;
      state.eeprom_i2c_address = EEPROM_I2C_ADDR1;
      state.eeprom_mem_address = 0;
    }

  }

  return switched_bank;

}


void write_xeprom_1(uint8_t byt)
{
  uint8_t messageBuf[6];

  messageBuf[0] = (state.eeprom_i2c_address << TWI_ADR_BITS) ;
  messageBuf[1] = (state.eeprom_mem_address >> 8) & 0xff;
  messageBuf[2] = state.eeprom_mem_address & 0xff;
  messageBuf[3] = byt;
  state.eeprom_mem_address += 1;
  USI_TWI_Start_Read_Write( messageBuf, 4);
  _delay_ms(I2C_DELAY_MS);

}

void write_xeprom_2(uint8_t byt0, uint8_t byt1)
{
  uint8_t messageBuf[6];

  messageBuf[0] = (state.eeprom_i2c_address << TWI_ADR_BITS) ;
  messageBuf[1] = (state.eeprom_mem_address >> 8) & 0xff;
  messageBuf[2] = state.eeprom_mem_address & 0xff;
  messageBuf[3] = byt0;
  messageBuf[4] = byt1;
  state.eeprom_mem_address += 2;
  USI_TWI_Start_Read_Write( messageBuf, 5);
  _delay_ms(I2C_DELAY_MS);

}

void write_eeprom(uint16_t word)
{
  uint8_t messageBuf[6];

  messageBuf[0] = (state.eeprom_i2c_address << TWI_ADR_BITS) ;
  messageBuf[1] = (state.eeprom_mem_address >> 8) & 0xff;
  messageBuf[2] = state.eeprom_mem_address & 0xff;
  messageBuf[3] = (word >> 8) & 0xff;
  messageBuf[4] = word & 0xff;
  state.eeprom_mem_address += 2;
  USI_TWI_Start_Read_Write( messageBuf, 5);
  _delay_ms(I2C_DELAY_MS);

}

void read_eeprom(uint8_t *buf, uint8_t n)
{
  uint8_t i;
  uint8_t messageBuf[6];

  messageBuf[0] = (state.eeprom_i2c_address << TWI_ADR_BITS) ;
  messageBuf[1] = (state.eeprom_mem_address >> 8) & 0xff;
  messageBuf[2] = state.eeprom_mem_address & 0xff;
  USI_TWI_Start_Read_Write( messageBuf, 3);
  _delay_ms(I2C_DELAY_MS);

  for (i=0; i<n; i++)
  {
    messageBuf[0] = (state.eeprom_i2c_address << TWI_ADR_BITS) | (1<<TWI_READ_BIT);
    messageBuf[1] = 1;
    USI_TWI_Start_Read_Write( messageBuf, 2);
    buf[i] = messageBuf[1];
    _delay_ms(I2C_DELAY_MS);
  }

}

// write to eeprom address pointers
// updates address_pointer_seq
void update_xeprom_address_pointers(void)
{
  uint16_t addr;
  uint8_t messageBuf[8];

  state.address_pointer_seq++;      // sequence
  state.address_pointer_pos++;      // position in first n address pointer locations
  state.address_pointer_pos %= state.n_address_pointer;

  addr = HEADER_SIZE + ( ADDRESS_POINTER_SIZE * state.address_pointer_pos );

  messageBuf[0] = (EEPROM_I2C_ADDR0 << TWI_ADR_BITS) ;
  messageBuf[1] = (addr >> 8) & 0xff;
  messageBuf[2] = addr & 0xff;
  messageBuf[3] = state.address_pointer_seq;
  messageBuf[4] = state.eeprom_i2c_address;
  messageBuf[5] = (state.eeprom_mem_address >> 8) & 0xff;
  messageBuf[6] = state.eeprom_mem_address & 0xff;
  USI_TWI_Start_Read_Write( messageBuf, 7);
  _delay_ms(I2C_DELAY_MS);

}


// load current eeprom address variables from eeprom address pointers
//
void setup_xeprom_address_vars()
{
  uint8_t i;
  uint8_t is_wrap = 0;
  uint8_t max_addr = 0;
  uint8_t aux_addr = 0;
  uint8_t address_pointer[ADDRESS_POINTER_SIZE];

  uint8_t address_pointer_seq = 0xff;
  uint8_t address_pointer_pos = 0xff;
  uint8_t eeprom_i2c_address  = 0xff;
  uint16_t eeprom_mem_address = 0xffff;

  for (i=0; i < state.n_address_pointer; i++)
  {
    state.eeprom_mem_address = HEADER_SIZE + ( ADDRESS_POINTER_SIZE * i);
    read_eeprom( address_pointer, ADDRESS_POINTER_SIZE );

    // if we haven't read a max address yet, just ignore
    // otherwise check if we've wrapped
    if (  (max_addr > address_pointer[0]) &&
         ((max_addr - address_pointer[0]) > (state.n_address_pointer - 1)) )
      is_wrap = 1;

    if ( max_addr <= address_pointer[0] )
    {
      max_addr = address_pointer[0];
      if (!is_wrap)
      {
        address_pointer_seq = max_addr;
        address_pointer_pos = i;
        eeprom_i2c_address = address_pointer[1];
        eeprom_mem_address = address_pointer[2] << 8;
        eeprom_mem_address |= address_pointer[3];
      }
    }

    // save address in case we've wrapped
    if ( (address_pointer[0] < (256 - state.n_address_pointer)) &&
         (aux_addr <= address_pointer[0]) )
    {
      aux_addr = address_pointer[0];
      if (is_wrap)
      {
        address_pointer_seq = aux_addr;
        address_pointer_pos = i;
        eeprom_i2c_address = address_pointer[1];
        eeprom_mem_address = address_pointer[2] << 8;
        eeprom_mem_address |= address_pointer[3];
      }
    }

  }

  // generic initialization condition
  // for example, if the memory has been flashed
  if ( (address_pointer_seq == 0xff) &&
       (eeprom_i2c_address == 0xff) )
  {
    state.address_pointer_seq = 0;
    state.eeprom_i2c_address = DEFAULT_EEPROM;
    state.eeprom_mem_address = HEADER_SIZE + (ADDRESS_POINTER_SIZE * state.n_address_pointer);

    state.default_init = 1;
  }
  else
  {
    state.address_pointer_seq = address_pointer_seq;
    state.address_pointer_pos   = address_pointer_pos;
    state.eeprom_i2c_address  = eeprom_i2c_address;
    state.eeprom_mem_address  = eeprom_mem_address;
  }

}

void setup_boot_information(void)
{
  uint8_t i;
  uint8_t messageBuf[6];
  uint8_t header[HEADER_SIZE];

  messageBuf[0] = (EEPROM_I2C_ADDR0 << TWI_ADR_BITS) ;
  messageBuf[1] = 0;
  messageBuf[2] = 0;
  USI_TWI_Start_Read_Write( messageBuf, 3);
  _delay_ms(I2C_DELAY_MS);

  for (i=0; i<HEADER_SIZE; i++)
  {
    messageBuf[0] = (EEPROM_I2C_ADDR0 << TWI_ADR_BITS) | (1<<TWI_READ_BIT);
    messageBuf[1] = 1;
    USI_TWI_Start_Read_Write( messageBuf, 2);
    _delay_ms(I2C_DELAY_MS);

    header[i] = messageBuf[1];
  }

  state.eeprom_policy  = (header[0] & 0xf0) >> 4;
  state.log_frequency  = (header[0] & 0x0f);
  state.n_address_pointer = header[1];

  if ( (state.eeprom_policy == 0xf) &&
       (state.log_frequency == 0xf) &&
       (state.n_address_pointer == 0xff) )
  {
    state.eeprom_policy = DEFAULT_EEPROM_POLICY;
    state.log_frequency = DEFAULT_FREQ;
    state.n_address_pointer = DEFAULT_N_ADDRESS;

    state.default_init = 1;
  }

}

void write_record( uint8_t record_type, uint8_t record_len, uint8_t *buf)
{
  uint8_t i;

  update_xeprom_counters( record_len + 1 );
  if (state.eeprom_full) return;

  write_xeprom_1( record_type | record_len );
  for (i=0; i < record_len; i++)
    write_xeprom_1( buf[i] );
  
}

void setup_default(void)
{
  uint8_t i;

  state.eeprom_i2c_address = DEFAULT_EEPROM;
  state.eeprom_mem_address = 0;

  write_xeprom_1( DEFAULT_EEPROM_POLICY | DEFAULT_FREQ );
  write_xeprom_1( DEFAULT_N_ADDRESS );

  for (i=2; i<HEADER_SIZE; i++)
    write_xeprom_1(0);

  for (i=0; i<DEFAULT_N_ADDRESS; i++)
  {
    write_xeprom_1(i);
    write_xeprom_1(DEFAULT_EEPROM);
    write_eeprom( HEADER_SIZE + (DEFAULT_N_ADDRESS*ADDRESS_POINTER_SIZE) );
  }

}

// convert RTC date into a string
void get_rtc_str_date(uint8_t *dt)
{

  uint8_t i;
  uint8_t messageBuf[6];
  uint8_t buf[10];
  uint8_t h;

  messageBuf[0] = (RTC_I2C_ADDR << 1) ;
  messageBuf[1] = 0;
  USI_TWI_Start_Read_Write( messageBuf, 2);
  _delay_ms(I2C_DELAY_MS);

  for (i=0; i<7; i++)
  {
    messageBuf[0] = (RTC_I2C_ADDR << 1) | 1;
    messageBuf[1] = 0xbe;
    messageBuf[2] = 0xef;  // dummy values for read
    USI_TWI_Start_Read_Write( messageBuf, 2);
    _delay_ms(I2C_DELAY_MS);

    buf[i] = messageBuf[1];
  }

  // we're assuming we live in the future
  //year
  dt[0] = '2';
  dt[1] = '0';
  dt[2] = ((buf[6] & 0xf0) >> 4) + '0';
  dt[3] =  (buf[6] & 0x0f) + '0';

  //month
  dt[4] = ((buf[5] & 0x10) >> 4) + '0';
  dt[5] =  (buf[5] & 0x0f) + '0';

  //date
  dt[6] = ((buf[4] & 0x30) >> 4) + '0';
  dt[7] =  (buf[4] & 0x0f) + '0';

  //hour
  if ( buf[2] & 0x40 ) // check 12 / 24' bit
  {
    // convert from bcd to integer
    h = (((buf[2] & 0x10) >> 4)*10) + (buf[2] & 0x0f);
    if ( buf[2] & 0x20 ) // am' / pm flag
      h += 12;

    // convert to ascii
    dt[8] = (h / 10) + '0';
    dt[0] = (h % 10) + '0';
    
  }
  else  //mil time
  {
    dt[8] = ((buf[2] & 0x30) >> 4) + '0';
    dt[9] =  (buf[2] & 0x0f) + '0';
  }

  //minute
  dt[10] = ((buf[1] & 0x70) >> 4) + '0';
  dt[11] =  (buf[1] & 0x0f) + '0';


  // seconds
  dt[12] = ((buf[0] & 0x70) >> 4) + '0';
  dt[13] =  (buf[0] & 0x0f) + '0';


}
