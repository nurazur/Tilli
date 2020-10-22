/*******************************************************************************
 * Copyright (c) 2015 Thomas Telkamp and Matthijs Kooijman
 * Copyright (c) 2018 Terry Moore, MCCI
 *
 * Permission is hereby granted, free of charge, to anyone
 * obtaining a copy of this document and accompanying files,
 * to do whatever they want with them without any restriction,
 * including, but not limited to, copying, modification and redistribution.
 * NO WARRANTY OF ANY KIND IS PROVIDED.
 *
 * This example sends a valid LoRaWAN packet with payload "Hello,
 * world!", using frequency and encryption settings matching those of
 * the The Things Network.
 *
 * This uses ABP (Activation-by-personalisation), where a DevAddr and
 * Session keys are preconfigured (unlike OTAA, where a DevEUI and
 * application key is configured, while the DevAddr and session keys are
 * assigned/generated in the over-the-air-activation procedure).
 *
 * Note: LoRaWAN per sub-band duty-cycle limitation is enforced (1% in
 * g1, 0.1% in g2), but not the TTN fair usage policy (which is probably
 * violated by this sketch when left running for longer)!
 *
 * To use this sketch, first register your application and device with
 * the things network, to set or generate a DevAddr, NwkSKey and
 * AppSKey. Each device should have their own unique values for these
 * fields.
 *
 * Do not forget to define the radio type correctly in
 * arduino-lmic/project_config/lmic_project_config.h or from your BOARDS.txt.
 *
 *******************************************************************************/
/*******************************************************************************
  Adapted by nurazur for Tilly
  same license as above applies. If any doubt, GNU GPL Version 3 is to be used.
********************************************************************************/

#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>

#include "ttn-abp-config.h"

/******  Serial Port  ****/
// serial port needs to be disabled when using BME280 with ATmega328P devices, i.e. TiNo hardware
#define USE_SERIAL
/*************************/


// These callbacks are only used in over-the-air activation, so they are
// left empty here (we cannot leave them out completely unless
// DISABLE_JOIN is set in arduino-lmic/project_config/lmic_project_config.h,
// otherwise the linker will complain).
void os_getArtEui (u1_t* buf) { (void) buf;}
void os_getDevEui (u1_t* buf) { (void) buf;}
void os_getDevKey (u1_t* buf) { (void) buf;}


static osjob_t sendjob;



// Pin mapping
const lmic_pinmap lmic_pins = {
    .nss = RFM95_CS,                       // chip select on TiLLi
    .rxtx = LMIC_UNUSED_PIN,
    .rst = LMIC_UNUSED_PIN,          // reset pin
    .dio = {RFM95_DIO0, RFM95_DIO1, RFM95_DIO2},  //DIO0 und DIO1 on TiLLi
};

#include <PinChangeInterrupt.h>
uint8_t event_triggered = 0;

// ISR for the Pin change Interrupt
void wakeUp0() { event_triggered |= 0x1; }
void wakeUp1() { event_triggered |= 0x2; }
void wakeUp2() { event_triggered |= 0x4; }
void wakeUp3() { event_triggered |= 0x8; }

/*****************************************************************************/
/***                       Payload Definition                              ***/
/*****************************************************************************/

/* Payload structure is dependent on Sensor type */

static uint16_t count;


/*****************************************************************************/
/***                            Sensor Drivers                             ***/
/*****************************************************************************/
// choose which sensor to use in ttn-abp-config.h

// I2C Sensors connect through SoftwareWire, so that SCL and SDA can be any pin
// Add pullup resistors between SDA/VCC and SCL/VCC if not yet provided on Module board


// if I2C bus has no pullups externally, you can use internal Pullups instead.
// Internal pullup resistors are ~30kOHm. Since we are clocking slowly,
// it works. However, they need to be disabled in sleep mode, because SoftwareWire
// keeps them enabled even if i2c is ended with end()

//#define USE_I2C_PULLUPS  true // use this for HTU21D in case no Pullups are mounted.
#define USE_I2C_PULLUPS false // use this in case external Pullups are used.

/**********      HTU21D       ******/
#ifdef USE_HTU21D

typedef struct
{
   uint8_t  event;
   uint16_t count;
   uint16_t vcc;
   uint8_t  brightness;
   uint16_t temp;   // Temperature reading
   uint8_t humidity;    // Humidity reading
}  Payload;


#include "SoftwareWire.h"
SoftwareWire i2c(SDAPIN, SCLPIN, USE_I2C_PULLUPS);

#include "HTU21D_SoftwareWire.h"
HTU21D_SoftI2C myHTU21D(&i2c);

static void Start_HTU21D(void)
{
    i2c.beginTransmission (HTDU21D_ADDRESS);
    if (i2c.endTransmission() == 0)
    {
        //Serial.println ("Found HTU21D");
        delay(100);
        myHTU21D.begin();
        myHTU21D.setResolution(HTU21D_RES_RH10_TEMP13);
    }
    else
    {
        #ifdef USE_SERIAL 
        Serial.println ("No HTU21D found");
        #endif
    }
}

static void Measure_HTU21D(Payload& msg);
static void Measure_HTU21D(Payload& msg)
{
    pinMode(node.I2CPowerPin, OUTPUT);  // set power pin for Sensor to output
    digitalWrite(node.I2CPowerPin, HIGH); // turn Sensor on
    i2c.begin();
    myHTU21D.begin();
    delay(50);
    float temperature = myHTU21D.readTemperature();
    msg.temp = floor(temperature * 100 + 10000.5);
    msg.humidity = myHTU21D.readCompensatedHumidity(temperature) * 2;
    if (USE_I2C_PULLUPS) // prepare for low power state
    {
        pinMode(SDAPIN, INPUT);
        pinMode(SCLPIN, INPUT);
    }
    digitalWrite(node.I2CPowerPin, LOW); // turn Sensor off
}
#endif

/**********      BME280     **********/
#ifdef USE_BME280

// Flash size not sufficient to acommodate serial debug outputs. 
#ifdef USE_SERIAL
#undef USE_SERIAL
#endif
typedef struct
{
   uint8_t  event;
   uint16_t count;
   uint16_t vcc;
   uint8_t  brightness;
   uint16_t temp;   // Temperature reading
   uint8_t humidity;    // Humidity reading
   uint16_t pressure; // absolute pressure - 500 hpa
}  Payload;

#include "SoftwareWire.h"
SoftwareWire i2c(SDAPIN, SCLPIN, USE_I2C_PULLUPS);

#include <BME280_SoftwareWire.h>
BME280_SoftwareWire bme(&i2c);


static void Start_BME280(void)
{
    i2c.beginTransmission (0x76);

    if (i2c.endTransmission() == 0)
    {
        #ifdef USE_SERIAL
        Serial.println ("Found BME280");Serial.flush();
        #endif
    }
}

static void Measure_BME280(Payload& msg);
static void Measure_BME280(Payload& msg)
{
    float hum(NAN), pres(NAN), temperature(NAN);
    BME280b::TempUnit tempUnit(BME280b::TempUnit_Celcius);
    BME280b::PresUnit presUnit(BME280b::PresUnit_hPa);

    pinMode(node.I2CPowerPin, OUTPUT);  // set power pin for Sensor to output
    digitalWrite(node.I2CPowerPin, HIGH);
    i2c.begin();
    delay(20);
    bme.begin();
    delay(125);

    bme.read(pres, temperature, hum, tempUnit, presUnit);
    digitalWrite(node.I2CPowerPin, LOW);
    msg.temp = floor(temperature * 100 + 10000.5);
    msg.humidity = (hum*2);
    msg.pressure = (pres - 500) *100;
    
    #ifdef USE_SERIAL
    Serial.print("Pressure: ");
    Serial.print(msg.pressure);
    Serial.print(" hPa, ");
    Serial.flush();
    #endif
    digitalWrite(node.I2CPowerPin, LOW);
}
#endif


/**********      DS18B20     **********/
#ifdef USE_DS18B20
typedef struct
{
   uint8_t  event;
   uint16_t count;
   uint16_t vcc;
   uint8_t  brightness;
   uint16_t temp;   // Temperature reading
}  Payload;

#include <DallasTemperature.h>       // GNU Lesser General Public License v2.1 or later
#include <OneWire.h>                 // license terms not clearly defined.

#define ONE_WIRE_BUS SDAPIN
#define ONE_WIRE_POWER node.I2CPowerPin

OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature ds18b20(&oneWire);

static uint8_t start_ds18b20(DallasTemperature *sensor, byte PowerPin);


// One-Wire DS18B20 start-up sequence
static uint8_t start_ds18b20(DallasTemperature *sensor, byte PowerPin)
{
    pinMode(PowerPin, OUTPUT); // set power pin for DS18B20 to output
    digitalWrite(PowerPin, HIGH); // turn DS18B20 sensor on
    delay(10); // Allow 10ms for the sensor to be ready
    sensor->begin();
    //sensor->setResolution(10); //Resolutiuon is 0.125 deg, absolutely sufficient!
    delay(10); // Allow 10ms for the sensor to be ready
    return sensor->getDeviceCount();
}

static void stop_ds18b20(byte PowerPin)
{
    digitalWrite(PowerPin, LOW); // turn Sensor off to save power
}


static void Start_DS18B20(void)
{
    //--------------------------------------------------------------------------
    // test if 1-wire devices are present
    //--------------------------------------------------------------------------
    pinMode(ONE_WIRE_POWER, OUTPUT); // set power pin for DS18B20 to output
    digitalWrite(ONE_WIRE_POWER, HIGH); // turn DS18B20 sensor on
    delay(10); // Allow 10ms for the sensor to be ready

    uint8_t num_devices = start_ds18b20(&ds18b20, ONE_WIRE_POWER);
    if (num_devices == 0)
    {
        //delete ds18b20;
        //ds18b20 = NULL;
        stop_ds18b20(ONE_WIRE_POWER);
        #ifdef USE_SERIAL
        Serial.print("no Dallas DS18B20 found\n\r");
        #endif

    }
    else
    {
        #ifdef USE_SERIAL
        Serial.print(num_devices, DEC);
        Serial.println(" DS18B20 devices found.");
        Serial.flush();
        #endif
    }
}

static void Measure_DS18B20(Payload& msg);
static void Measure_DS18B20(Payload& msg)
{
    float temperature;
    uint8_t num_devices = start_ds18b20(&ds18b20, ONE_WIRE_POWER);
    if (num_devices > 0)
    {
        ds18b20.requestTemperatures();

        temperature = ds18b20.getTempCByIndex(0);
        msg.temp = floor(temperature * 100 + 10000.5);
        #ifdef USE_SERIAL
        Serial.print("Temp: ");Serial.print(temperature); Serial.println(" degC");
        #endif
        stop_ds18b20(ONE_WIRE_POWER); // Turn off power Pin for DS18B20
    }
    delay(65); // add delay to allow wdtimer to increase in case its erroneous
    #ifdef USE_SERIAL
    for (uint8_t i =1; i< num_devices; i++)
    {
         Serial.print("Temp "); Serial.print(i); Serial.print(": "); Serial.print(ds18b20.getTempCByIndex(i)); Serial.println(" degC");
    }
    #endif
}
#endif

/*****************************************************************************/
/***                       Konfiguration                                   ***/
/*****************************************************************************/
#include <EEPROM.h>

static uint16_t checksum_crc16(uint8_t *data, uint8_t data_len)
{
    uint16_t crc = 0xFFFF;
    //uint8_t data_len = offsetof(Configuration, checksum);
    //uint8_t *data = (uint8_t*) &Cfg;

    if (data_len == 0)
        return 0;

    for (unsigned int i = 0; i < data_len; ++i)
    {
        uint16_t dbyte = data[i];
        crc ^= dbyte << 8;

        for (unsigned char j = 0; j < 8; ++j)
        {
            uint16_t mix = crc & 0x8000;
            crc = (crc << 1);
            if (mix)
                crc = crc ^ 0x1021;
        }
    }

    return crc;
}

bool verify_checksum_crc16(void)
{
    uint16_t cs = checksum_crc16((uint8_t*) &node, offsetof(NodeConfig, checksum)) ^ node.checksum;
    if (cs != 0)
        return false;
    return true;
}

void (*SoftReset)(void) = 0;

uint16_t LED_blink_time=0xff;

static void update_eeprom(void)
{
     node.checksum = checksum_crc16((uint8_t*) &node, offsetof(NodeConfig, checksum));
     EEPROM.put(0, node);
}

/*****************************************************************************/
/***                       Brightness                                      ***/
/*****************************************************************************/
static uint8_t brightness(uint8_t LedA, uint8_t LedK)
{
    uint16_t counter;

    digitalWrite(LedA, LOW); // LED Anode is always in mode OUTPUT

    // charge LED in reverse mode
    pinMode(LedK, OUTPUT);
    digitalWrite(LedK, HIGH);
    delayMicroseconds(8);

    // Kathode to INPUT
    pinMode(LedK,INPUT);
    // wait and count until Kathode is LOW
    for (counter = 0; counter < 65535; counter++)
    {
    if (digitalRead(LedK)==0) break;
    delayMicroseconds(10);
    // 10 - 12 us good for yellow
    // 0 - 4 is good for blue
    // green and red is difficult, use 0?
    // 10 -16us good for white
    }

    // change pinmode of Kathode to OUTPUT, LOW so that LED can be used as LED
    init_Led(LedK);

    return (uint8_t)(counter >>8);
}

static void init_Led(uint8_t LedK)
{
    pinMode(LedK,OUTPUT);
    digitalWrite(LedK, LOW);
}

/*****************************************************************************/
/***                      Sleep Modes                                      ***/
/*****************************************************************************/

volatile uint16_t watchdog_counter;
volatile uint16_t pir_inhibit_counter;
bool pir_is_off=true;
ostime_t led_on_time;
bool watchdog_expired = false;
bool tx_is_complete = false;

// USE_CRYSTAL is a command-line option, do not change here unless you compile from the command line
#include "LowPower.h"
#ifdef USE_CRYSTAL

// interrupt on Timer 2 compare "A" completion
// requires a 32.768kHz Crystal
ISR(TIMER2_COMPA_vect)
{
    //watchdog_counter++;
    //pir_inhibit_counter++;
}

typedef enum 
{
  T_OFF,
  T_8SECONDS,
  T_2SECONDS,
  T_1SECOND,
  T_500MILLIS,
  T_250MILLIS,
  T_62_5MILLIS,
  T_8MILLIS
} prescaler;

static void setup_timer2(uint8_t st)
{
  // clock input to timer 2 from XTAL1/XTAL2
  ASSR = bit (AS2);
  while (ASSR & 0x1f);

  TCCR2A = bit (WGM21);     //WGM20 and WGM22 in TCCR2B is set 0, Mode of operation: CTC

  switch (st)
  {
      case T_OFF:
        TCCR2B  &= 0xF8; // timer off, CS0=0, CS1=0, CS2=0
        break;
        
    case T_8SECONDS:
        TCCR2B = bit (CS22) | bit (CS21) | bit (CS20);   //111b
        break;

    case T_2SECONDS:
        TCCR2B = bit (CS22) | bit (CS21);   // 110b
        break;

    case T_1SECOND:    // 128
        TCCR2B = bit (CS22) | bit (CS20); //101b
        break;

    case T_500MILLIS:// 64
        TCCR2B = bit (CS22); //100b
        break;

    case T_250MILLIS:// 32
        TCCR2B = bit (CS21) | bit (CS20); //011b
        break;
    case T_62_5MILLIS: // 8
        TCCR2B = bit (CS21); //010b
        break;
        
    case T_8MILLIS: // 1
        TCCR2B = bit (CS21) | bit (CS20); //001b
        break;
  }

  while (ASSR & 0x08);   // wait until OCR2A can be updated
  OCR2A =  255;         // count to 255 (zero-relative)


  while (ASSR & 0x1f);   // update is busy

  // enable timer interrupts
  TIMSK2 |= bit (OCIE2A);
}

//Option with internal RC Oscillator Watchdog timer
#else
#endif


/*****************************************************************************/
/*
     Read VCC by taking measuring the 1.1V reference and taking VCC as reference voltage.
     set the reference to Vcc and the measurement to the internal 1.1V reference
*/
/*****************************************************************************/
unsigned int readVcc(void) {

  #if defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)  || defined (ARDUINO_AVR_ATmega1284) || defined(ARDUINO_AVR_ATmega644)
      //Serial.println("Atmega1284");
      ADMUX = _BV(REFS0) | _BV(MUX4) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  #elif defined (__AVR_ATtiny24__) || defined(__AVR_ATtiny44__) || defined(__AVR_ATtiny84__)
      ADMUX = _BV(MUX5) | _BV(MUX0);
  #elif defined (__AVR_ATtiny25__) || defined(__AVR_ATtiny45__) || defined(__AVR_ATtiny85__)
      ADMUX = _BV(MUX3) | _BV(MUX2);
  #else
      ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  #endif
  delay(2); // Wait for Vref to settle
  ADCSRA |= _BV(ADSC); // Start conversion
  while (bit_is_set(ADCSRA,ADSC)); // measuring
  uint8_t low  = ADCL; // must read ADCL first - it then locks ADCH
  uint8_t high = ADCH; // unlocks both
  uint16_t result = (high<<8) | low;
  return result;
}

unsigned int vcc_reading=1;

/*****************************************************************************/
/*
handler for PIR events. When an PIR event is triggered, the PIR is switched off for
PIR_INHIBIT_DELAY counts (1 count =8s). After this, the PIR VCC ist switched on again.
This is to save power in situations where the PIR would constantly trigger.

There is a Heartbeat funktion which sends a "I am alive" packet every SENDDELAY counts (1 count = 8s)
Note that SENDDELAY cannot be 0 because this would turn off the timner!
The timer is needed for the PIR_INHIBIT_DELAY count.
*/

static void wake_from_sleep_service_routine(void)
{
    static bool timer2_setup_done = false;
    static bool pir_inhibit_expired = false;
    Payload msg;

    while (1)
    {
        #ifdef USE_CRYSTAL
        //********    go sleep          *****/
        if (node.Flag_LED_blink_active && node.Flag_LED_enable)
        {
         /***** blink for 8 seconds *****/
            setup_timer2(T_500MILLIS);
            for (uint8_t j=0; j<16; j++)
            {
                digitalWrite(node.LedPin, !digitalRead(node.LedPin));
                LowPower.powerSave(SLEEP_FOREVER, ADC_OFF, BOD_OFF, TIMER2_ON);
            }
            if (LED_blink_time != 0xff)
            {
                LED_blink_time--;
                if (LED_blink_time ==0)
                {
                    node.Flag_LED_blink_active =0;
                    digitalWrite(node.LedPin, LOW);
                    setup_timer2(T_8SECONDS);
                }
            }
        }
        else
        {
            if (node.SendDelay != 0)
                LowPower.powerSave(SLEEP_FOREVER, ADC_OFF, BOD_OFF, TIMER2_ON);
            else
            {
                // only wake up from external pin change interrupt
                timer2_setup_done = true;
                TCCR2B &= 0xF8; // timer 2 off, CS0=0, CS1=0, CS2=0
                LowPower.powerDown(SLEEP_FOREVER, ADC_OFF, BOD_OFF);
            }
        }

        //*********** after wake up...   *****/

        // workaround for timer2 setup failure (registers read OK but timer runs fast)
        // this is a one-time task after the board has started.
        if (!timer2_setup_done)
        {
            setup_timer2(T_8SECONDS); // set it up again. Cures the problem.
            watchdog_counter = 0;
            timer2_setup_done =  true;
            continue;
        }


        #else // low power RC Oscillator
        //********    go sleep           *****/
        if (node.Flag_LED_blink_active && node.Flag_LED_enable)
        {
            for (uint8_t j=0; j<8; j++)
            {
                digitalWrite(node.LedPin, !digitalRead(node.LedPin));
                LowPower.powerDown(SLEEP_1S, ADC_OFF, BOD_OFF);
            }

            if (LED_blink_time != 0xff)
            {
                LED_blink_time--;
                if (LED_blink_time ==0)
                {
                    node.Flag_LED_blink_active =0;
                    digitalWrite(node.LedPin, LOW);
                }
            }
        }
        else
        {
            LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);
        }
        //*********** after wake up...   *****/
        #endif
        if (!event_triggered)
        {
            watchdog_counter++;
            pir_inhibit_counter++;
        }


        // switch ON PIR after dead time
        pir_inhibit_expired = (pir_inhibit_counter >= node.PIRDisableDelay) && pir_is_off && node.Flag_PIR_enable;
        if (pir_inhibit_expired) // Totzeit fuer PIR ist abgelaufen
        {
            pir_inhibit_counter = 0;
            pir_is_off = false;
            digitalWrite(node.PIRVccPin, HIGH);
            pir_inhibit_expired = false;
            // need to wait for 2.5s because data pin goes high for 2.5s when PIR ist switched ON
            LowPower.powerDown(SLEEP_500MS , ADC_OFF, BOD_OFF);
            LowPower.powerDown(SLEEP_2S, ADC_OFF, BOD_OFF);
            #ifdef USE_SERIAL
            Serial.println("PIR aktiv."); Serial.flush();
            #endif
            event_triggered=0;
            continue;
        }

        // Send Heartbeat
        watchdog_expired = ((watchdog_counter >= node.SendDelay)  && (node.SendDelay !=0));
        if (watchdog_expired)
        {
            #ifdef USE_SERIAL
            Serial.println("heartbeat"); Serial.flush();
            #endif
            watchdog_counter =0;
            watchdog_expired = false;

            if(node.Flag_heartbeat_enable)
            {
            msg.event = 1;
            msg.vcc = node.VccReference / vcc_reading;
            msg.count = count++;
            msg.brightness = brightness(node.LedPin, node.LedPin_K);
            #ifdef USE_HTU21D
            Measure_HTU21D(msg);
            #elif defined USE_BME280
            Measure_BME280(msg);
            #elif defined USE_DS18B20
            Measure_DS18B20(msg);
            #endif
            //if (node.Flag_heartbeat_enable) geht so nicht!
            do_send(&sendjob, (uint8_t*)&msg, sizeof(msg));
            return;
            }
            else
            {
                continue;
            }
        }


        // send packet if an event has happened.
        if (event_triggered)
        {
            if (event_triggered & 0x1)
            {
                if (node.Flag_LED_enable) digitalWrite(node.LedPin, HIGH);
                #ifdef USE_SERIAL 
                Serial.println("PCI0");Serial.flush();
                #endif
            }
            if (event_triggered & 0x2) //PCI1 reserved for PIR sensor
            {
                if (pir_is_off)
                {
                    continue; // triggered during dead time
                }
                else
                {
                    if (node.Flag_LED_enable) digitalWrite(node.LedPin, HIGH);
                    #ifdef USE_SERIAL
                    Serial.println("PIR");Serial.flush();
                    #endif
                    pir_inhibit_counter = 0;
                    pir_inhibit_expired = false;
                    pir_is_off = true;
                    digitalWrite(node.PIRVccPin, LOW);
                }
            }
            if (event_triggered & 0x4)
            {
                // not yet implemented
            }
            if (event_triggered & 0x8)
            {
                // not yet implemented
            }


            uint8_t *pnode = (uint8_t*) &node;
            msg.event = (event_triggered &0x0F) << 1;
            if (pnode[0] & msg.event)
            {
                msg.vcc = node.VccReference / vcc_reading;
                msg.count = count++;
                msg.brightness = brightness(node.LedPin, node.LedPin_K);
                #ifdef USE_HTU21D
                Measure_HTU21D(msg);
                #elif defined USE_BME280
                Measure_BME280(msg);
                #elif defined USE_DS18B20
                Measure_DS18B20(msg);
                #endif
                event_triggered =0;
                do_send(&sendjob, (uint8_t*)&msg, sizeof(msg));
                return;
            }
            else
            {
                continue;
            }

        }
    }
}

/*
TX-RX transaction flags - report back to user
enum { TXRX_ACK    = 0x80,   // confirmed UP frame was acked
       TXRX_NACK   = 0x40,   // confirmed UP frame was not acked
       TXRX_NOPORT = 0x20,   // set if a frame with a port was RXed, clr if no frame/no port
       TXRX_PORT   = 0x10,   // set if a frame with a port was RXed, LMIC.frame[LMIC.dataBeg-1] => port
       TXRX_LENERR = 0x08,   // set if frame was discarded due to length error.
       TXRX_PING   = 0x04,   // received in a scheduled RX slot
       TXRX_DNW2   = 0x02,   // received in 2dn DN slot
       TXRX_DNW1   = 0x01,   // received in 1st DN slot
};
*/

void receive()
{
    //Serial.print("flags: 0x"); Serial.println(LMIC.txrxFlags, HEX);
    u1_t* data = LMIC.frame + LMIC.dataBeg;
    #ifdef USE_SERIAL
    Serial.print("Data: ");
    
    for (uint8_t i=0; i< LMIC.dataLen; i++)
    {
        Serial.print(data[i], HEX); Serial.print(" ");
    }
    Serial.println();
    #endif
    uint8_t i=0;
    while (i<LMIC.dataLen)
    {
        if (data[i] & 0x80) // is a instruction
        {
            switch (data[i])
            {
                case 0x80: // LED off
                    digitalWrite(node.LedPin, LOW);
                    node.Flag_LED_blink_active=0;
                    break;
                case 0x81: // blink
                    node.Flag_LED_blink_active=1;
                    LED_blink_time = 0xff;
                    if (++i < LMIC.dataLen)
                        LED_blink_time = data[i];
                    if (++i < LMIC.dataLen)
                        LED_blink_time += data[i]<<8;
                    //Serial.println(LED_blink_time);
                    break;
                case 0x82: // enable / disable Interrupts , GPIO's and heartbeat
                    if (++i < LMIC.dataLen)
                    {
                        uint8_t *pnode = (uint8_t*) &node;
                        pnode[0] = (pnode[0] &0xE0) | (data[i] & 0x3F);
                    }
                    break;
                case 0xFD:
                    {
                        // invalidate EEPROM
                        uint16_t cs;
                        EEPROM.get(offsetof(NodeConfig, checksum), cs);
                        cs ^= 0xff;
                        EEPROM.put(offsetof(NodeConfig, checksum), cs);
                    }
                    //fall through
                case 0x88: // soft reset
                    SoftReset();
                    break;
                case 0xD5: // turn off node, storage condition
                    PCICR =0; // detach Pin change interrupts
                    // sleep with no timer
                    TCCR2B &= 0xF8; // timer 2 off, CS0=0, CS1=0, CS2=0
                    LowPower.powerDown(SLEEP_FOREVER, ADC_OFF, BOD_OFF);
                    //LowPower.powerSave(SLEEP_FOREVER, ADC_OFF, BOD_OFF, TIMER2_OFF);
                    break;
                default:
                    break;
            }
        }
        else // is configuration
        {
            uint8_t *pNode = (uint8_t*) &node;
            for (i=data[0]&0x3f; i<LMIC.dataLen; i++)
            {
                pNode[i] = data[i]; // copy all data beginning at <offset>
            }
            if (data[0] & 0x40)
            {
                update_eeprom();
            }
        }
        i++;
    }
}


/* Event types for event callback
EV_SCAN_TIMEOUT=1, EV_BEACON_FOUND,
             EV_BEACON_MISSED, EV_BEACON_TRACKED, EV_JOINING,
             EV_JOINED, EV_RFU1, EV_JOIN_FAILED, EV_REJOIN_FAILED,
             EV_TXCOMPLETE, EV_LOST_TSYNC, EV_RESET,
             EV_RXCOMPLETE, EV_LINK_DEAD, EV_LINK_ALIVE, EV_SCAN_FOUND,
             EV_TXSTART, EV_TXCANCELED, EV_RXSTART, EV_JOIN_TXCOMPLETE };
*/


void onEvent (ev_t ev) {
    #ifdef USE_SERIAL
    Serial.print(os_getTime());
    Serial.print(": ");
    #endif
    switch(ev) {
        case EV_SCAN_TIMEOUT:
        case EV_BEACON_FOUND:
        case EV_BEACON_MISSED:
        case EV_BEACON_TRACKED:
        case EV_JOINING:
        case EV_JOINED:
        case EV_JOIN_FAILED:
        case EV_REJOIN_FAILED:
            //Serial.print("EV: "); Serial.println(ev);
            break;
        case EV_TXCOMPLETE:
            #ifdef USE_SERIAL
            Serial.println(F("EV_TXCOMPLETE"));
            if (LMIC.txrxFlags & TXRX_ACK) Serial.println(F("txrx_ack"));
            #endif
            if (LMIC.dataLen)
            {
              // data have been received!
               receive();
            }
            tx_is_complete = true; // notify main loop
            #ifdef USE_SERIAL
            Serial.flush();
            #endif
            break;
        case EV_LOST_TSYNC:
        case EV_RESET:
        case EV_RXCOMPLETE:
        case EV_LINK_DEAD:
        case EV_LINK_ALIVE:
        case EV_TXSTART:
        case EV_TXCANCELED:
        case EV_JOIN_TXCOMPLETE:
            #ifdef USE_SERIAL
            Serial.print("EV: "); Serial.println(ev);
            #endif
            break;
        case EV_RXSTART:
            /* do not print anything -- it wrecks timing */
            break;
        default:
            #ifdef USE_SERIAL
            Serial.print(F("Unknown event: "));
            Serial.println((unsigned) ev);
            #endif
            break;
    }
}

void do_send(osjob_t* j, uint8_t* data, uint8_t datalen){
    // Check if there is not a current TX/RX job running
    if (LMIC.opmode & OP_TXRXPEND) 
    {
        #ifdef USE_SERIAL
        Serial.println(F("OP_TXRXPEND, not sending"));
        #endif
    } else {
        // Prepare upstream data transmission at the next possible time.
        LMIC_setTxData2(1, data, datalen, 0);
        #ifdef USE_SERIAL
        Serial.println(F("Packet queued"));
        #endif
    }
    // Next TX is scheduled after TX_COMPLETE event.
}

void setup() {
    while (!Serial); // wait for Serial to be initialized
    #ifdef USE_SERIAL
    Serial.begin(SERIALBAUD);
    #endif
    delay(100);
    #ifdef USE_SERIAL
    Serial.println(F("Start"));
    #endif

    EEPROM.get(0, node);
    //Serial.println(node.checksum, HEX);
    //Serial.println(checksum_crc16((uint8_t*) &node, offsetof(NodeConfig, checksum)),HEX );
    //Serial.println(sizeof(NodeConfig));

    
    #ifdef USE_SERIAL
    Serial.print ("CRC ");
    #endif
    if (true) // for debug and test
    //if (!verify_checksum_crc16()) // for production
    {
        // calculate checksum and put configuration into EEPROM
        #ifdef USE_SERIAL
        Serial.println("bad");
        #endif
        {
            NodeConfig nNode;
            node = nNode;
        }

        update_eeprom();
    }
    else
    {
        #ifdef USE_SERIAL
        Serial.println("OK");
        #endif
    }

    pinMode(node.LedPin, OUTPUT);
    if(node.LedPin_K >=0)
    {
         init_Led(node.LedPin_K);
    }

    



    #ifdef USE_CRYSTAL
        #ifdef USE_SERIAL
        Serial.println("crystal timer");
        #endif
    setup_timer2(T_8SECONDS);
    #else
        #ifdef USE_SERIAL
        Serial.println("Using WDT");
        #endif
    #endif


    //Serial.println(node.checksum,HEX);
    //Serial.println(node.Flag_LED_enable);


    /* for TEST only!
        node.Flag_heartbeat_enable=1;
        node.Flag_LED_enable=1;
        node.Flag_LED_blink_active=0;
        node.Flag_PIR_enable=1;
        node.Flag_reserved =0;
        node.checksum = checksum_crc16((uint8_t*) &node, offsetof(NodeConfig, checksum));
        EEPROM.put(0, node);
    */
    /*
    Serial.print("Flag_heartbeat_enable "); Serial.println(node.Flag_heartbeat_enable);
    Serial.print("Flag_PCI0_enable "); Serial.println(node.Flag_PCI0_enable);
    Serial.print("Flag_PIR_enable "); Serial.println(node.Flag_PIR_enable);
    Serial.print("Flag_PCI2_enable "); Serial.println(node.Flag_PCI2_enable);
    Serial.print("Flag_PCI3_enable "); Serial.println(node.Flag_PCI3_enable);
    Serial.print("Flag_LED_enable "); Serial.println(node.Flag_LED_enable);
    Serial.print("Flag_LED_blink_active "); Serial.println(node.Flag_LED_blink_active);

    Serial.print("Flag_reserved "); Serial.println(node.Flag_reserved);
    */
    //uint8_t *pnode = (uint8_t*) &node;
    //Serial.print("Flags LSB "); Serial.println(pnode[0], HEX);
    //Serial.print("Flags LSB "); Serial.println(*((uint8_t*) &node), HEX);// ja man koennte das so machen, aber der Code ist unlesbar!
    //Serial.print("Flags MSB "); Serial.println(pnode[1], HEX);
    //Serial.print("LMIC size in RAM: "); Serial.println(sizeof(LMIC)); // LMIV measures 796 BYtes!
    #ifdef USE_SERIAL
    Serial.flush();
    #endif

    if (node.PCI0Pin >=0)
    {
        pinMode(node.PCI0Pin, node.PCI0Trig>>2);  // set the pin to input or input with Pullup
        attachPinChangeInterrupt(digitalPinToPinChangeInterrupt(node.PCI0Pin), wakeUp0, node.PCI0Trig&0x3);
    }
    
    if (node.PIRPin >=0  && node.PIRVccPin >=0) // THE PIR SENSOR, do not change these settings!
    {
        pinMode(node.PIRPin, INPUT);
        // turn PIR Sensor ON so that it works immediately after start up.
        // But do activate the interrupt AFTER this so that no event is triggered.
        pinMode(node.PIRVccPin, OUTPUT);
        digitalWrite(node.PIRVccPin, HIGH);
        pir_is_off = false;
        delay(100);
        attachPinChangeInterrupt(digitalPinToPinChangeInterrupt(node.PIRPin), wakeUp1, RISING);
    }
    else
    {
        node.Flag_PIR_enable =0;
    }
    
    if (node.PCI2Pin >=0)
    {
        pinMode(node.PCI2Pin, node.PCI2Trig>>2);  // set the pin to input or input with Pullup
        attachPinChangeInterrupt(digitalPinToPinChangeInterrupt(node.PCI2Pin), wakeUp2, node.PCI2Trig&0x3);
    }
    
    if (node.PCI3Pin >=0)
    {
        pinMode(node.PCI3Pin, node.PCI3Trig>>2);  // set the pin to input or input with Pullup
        attachPinChangeInterrupt(digitalPinToPinChangeInterrupt(node.PCI3Pin), wakeUp3, node.PCI3Trig&0x3);
    }

/***  Initialize I2C and HTU21D  ***/
    pinMode(node.I2CPowerPin, OUTPUT);  // set power pin for Sensor to output
    digitalWrite(node.I2CPowerPin, HIGH);
    delay(5);
    
    #ifdef USE_HTU21D
    Start_HTU21D();
    #elif defined USE_BME280
    Start_BME280();
    #elif defined USE_DS18B20
    Start_DS18B20();
    #endif
    digitalWrite(node.I2CPowerPin, LOW);
    if (USE_I2C_PULLUPS) // make i2c Pins inputs for sleep mode.
    {
        pinMode(SDAPIN, INPUT);
        #ifndef USE_DS18B20
        pinMode(SCLPIN, INPUT);
        #endif
    } 
    
    
    // LMIC init
    os_init();
    //Serial.println(1);
    // Reset the MAC state. Session and pending data transfers will be discarded.
    LMIC_reset();
    //Serial.println(2);
    // Set static session parameters. Instead of dynamically establishing a session
    // by joining the network, precomputed session parameters are be provided.
    #ifdef PROGMEM
    // On AVR, these values are stored in flash and only copied to RAM once.
    memcpy_P(LMIC.nwkKey, NWKSKEY, sizeof(NWKSKEY));
    memcpy_P(LMIC.artKey, APPSKEY, sizeof(APPSKEY));
    LMIC_setSession (0x13, DEVADDR, NULL, NULL);
    //Serial.println(3);
    #else
    // If not running an AVR with PROGMEM, just use the arrays directly
    LMIC_setSession (0x13, DEVADDR, NWKSKEY, APPSKEY);
    //Serial.println(4);
    #endif

    #if defined(CFG_eu868)
    // Set up the channels used by the Things Network, which corresponds
    // to the defaults of most gateways. Without this, only three base
    // channels from the LoRaWAN specification are used, which certainly
    // works, so it is good for debugging, but can overload those
    // frequencies, so be sure to configure the full frequency range of
    // your network here (unless your network autoconfigures them).
    // Setting up channels should happen after LMIC_setSession, as that
    // configures the minimal channel set. The LMIC doesn't let you change
    // the three basic settings, but we show them here.
    LMIC_setupChannel(0, 868100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(1, 868300000, DR_RANGE_MAP(DR_SF12, DR_SF7B), BAND_CENTI);      // g-band
    LMIC_setupChannel(2, 868500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(3, 867100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(4, 867300000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(5, 867500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(6, 867700000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(7, 867900000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(8, 868800000, DR_RANGE_MAP(DR_FSK,  DR_FSK),  BAND_MILLI);      // g2-band
    // TTN defines an additional channel at 869.525Mhz using SF9 for class B
    // devices' ping slots. LMIC does not have an easy way to define set this
    // frequency and support for class B is spotty and untested, so this
    // frequency is not configured here.
    #elif defined(CFG_us915) || defined(CFG_au915)
    // NA-US and AU channels 0-71 are configured automatically
    // but only one group of 8 should (a subband) should be active
    // TTN recommends the second sub band, 1 in a zero based count.
    // https://github.com/TheThingsNetwork/gateway-conf/blob/master/US-global_conf.json
    LMIC_selectSubBand(1);
    #elif defined(CFG_as923)
    // Set up the channels used in your country. Only two are defined by default,
    // and they cannot be changed.  Use BAND_CENTI to indicate 1% duty cycle.
    // LMIC_setupChannel(0, 923200000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);
    // LMIC_setupChannel(1, 923400000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);

    // ... extra definitions for channels 2..n here
    #elif defined(CFG_kr920)
    // Set up the channels used in your country. Three are defined by default,
    // and they cannot be changed. Duty cycle doesn't matter, but is conventionally
    // BAND_MILLI.
    // LMIC_setupChannel(0, 922100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_MILLI);
    // LMIC_setupChannel(1, 922300000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_MILLI);
    // LMIC_setupChannel(2, 922500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_MILLI);

    // ... extra definitions for channels 3..n here.
    #elif defined(CFG_in866)
    // Set up the channels used in your country. Three are defined by default,
    // and they cannot be changed. Duty cycle doesn't matter, but is conventionally
    // BAND_MILLI.
    // LMIC_setupChannel(0, 865062500, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_MILLI);
    // LMIC_setupChannel(1, 865402500, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_MILLI);
    // LMIC_setupChannel(2, 865985000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_MILLI);

    // ... extra definitions for channels 3..n here.
    #else
    # error Region not supported
    #endif

    // Disable link check validation
    LMIC_setLinkCheckMode(0);

    // TTN uses SF9 for its RX2 window.
    LMIC.dn2Dr = DR_SF9;

    // Set data rate and transmit power for uplink
    LMIC_setDrTxpow(node.DataRate, node.TxPower);

    //
    LMIC_setClockError(MAX_CLOCK_ERROR * 2 / 100);
    // Start job
    Payload msg;
    msg.event =0; //  0 = Restart Flag
    msg.count= count++;
    msg.vcc = readVcc(); // value to be used for calibration
    msg.brightness = brightness(node.LedPin, node.LedPin_K);
    #ifdef USE_BME280
    Measure_BME280(msg);
    #elif defined USE_HTU21D
    Measure_HTU21D(msg);
    #elif defined USE_DS18B20
    Measure_DS18B20(msg);
    #endif
    
    do_send(&sendjob, (uint8_t*)&msg, sizeof(msg));
}

void loop() {
    os_runloop_once();
    if (os_getTime() - led_on_time > 6250) //100 ms = 6250 * 16us
    {
        digitalWrite(node.LedPin, LOW);
    }
    if( tx_is_complete )
    {
        tx_is_complete = false;
        pir_inhibit_counter=0;
        wake_from_sleep_service_routine();
    }
}
