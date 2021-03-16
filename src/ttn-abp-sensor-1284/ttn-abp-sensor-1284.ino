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
  Adapted by nurazur for Tilli
  same license as above applies. If any doubt, GNU GPL Version 3 is to be used.
********************************************************************************/

// uncomment define below if you wish just to test the sensors without sending LoRa packages
//#define TESTLOOP

// Version: 1.0.0

#include "tilli0.h"
#include "calibrate.h"
#include "framecounter.h"

/******  Serial Port  ****/
#define USE_SERIAL
/*************************/

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
/***                   Calibration Module                                  ***/
/*****************************************************************************/
#define KEY     "TheQuickBrownFox"
//Calibration CalMode(node, &Serial);
Calibration CalMode(node, &Serial, (uint8_t*) KEY);

/*****************************************************************************/
/***                            Sensor Drivers                             ***/
/*****************************************************************************/
// choose which sensor to use in tilli0.h

// I2C Sensors connect through SoftwareWire, so that SCL and SDA can be any pin
// Add pullup resistors between SDA/VCC and SCL/VCC if not yet provided on Module board


// if I2C bus has no pullups externally, you can use internal Pullups instead.
// Internal pullup resistors are ~30kOHm. Since we are clocking slowly,
// it works. However, they need to be disabled in sleep mode, because SoftwareWire
// keeps them enabled even if i2c is ended with end()
// define USE_I2C_PULLUPS in tilli0.h for specific devices which require the use of internal pullips.
#ifndef USE_I2C_PULLUPS
    #define USE_I2C_PULLUPS  false // use this if HTU21D is mounted on board
#endif


/**********      HTU21D       ******/
#ifdef USE_HTU21D_INVERTED_I2C 
    #define USE_HTU21D
#endif

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
        #ifdef USE_SERIAL
        Serial.println ("Found HTU21D");
        #endif
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
    #ifdef USE_SERIAL
    Serial.print("Temp: ");Serial.print(temperature);
    Serial.print(" degC, Hum: "); Serial.print(msg.humidity/2.0);
    Serial.println();
    #endif
    digitalWrite(node.I2CPowerPin, LOW); // turn Sensor off
}
#endif

/**********      BME280     **********/
#ifdef USE_BME280

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
    Serial.print("Temp: ");Serial.print(temperature); Serial.print(" degC, ");
    Serial.print ("RH: ");Serial.print(hum);Serial.print(" %, ");
    Serial.print("Pressure: ");
    Serial.print(msg.pressure);
    Serial.print(" hPa, ");
    Serial.println();
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

void (*SoftReset)(void) = 0;

uint16_t LED_blink_time=0xff;

static void update_eeprom(void)
{
    CalMode.checksum();
    CalMode.put_config_to_eeprom();
}

/*****************************************************************************/
/***                       Brightness                                      ***/
/*****************************************************************************/

static uint8_t brightness_with_LDR(uint8_t LDRPin)
{
    uint8_t sensorValue=0;
    
    if (LDRPin >= 0)
    {
        pinMode(LDRPin, INPUT_PULLUP);
        delay(20);
        sensorValue = (uint8_t) ((1023 - analogRead(LDRPin)) >>2);
        pinMode(LDRPin, INPUT);
    }
    return sensorValue;
}


/*****************************************************************************/
/***                      Sleep Modes                                      ***/
/*****************************************************************************/

volatile uint16_t watchdog_counter=0;
volatile uint16_t pir_inhibit_counter=0;
bool pir_is_off=true;
ostime_t led_on_time;
bool watchdog_expired = false;
bool tx_is_complete = false;


// USE_CRYSTAL is a command-line option, do not change here unless you compile from the command line
//#define USE_CRYSTAL
#include "LowPower.h"
//#ifdef USE_CRYSTAL

// interrupt on Timer 2 compare "A" completion
// requires a 32.768kHz Crystal

ISR(TIMER2_COMPA_vect)
{
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
//#else
//#endif


/*****************************************************************************/
/*
     Read VCC by taking measuring the 1.1V reference and taking VCC as reference voltage.
     set the reference to Vcc and the measurement to the internal 1.1V reference
*/
/*****************************************************************************/
unsigned int readVcc(void);
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
// Copy Bytes from EEPROM into RAM
void bytes_eeprom2ram(uint8_t* dest, uint16_t eeprom_offset, uint16_t num_bytes)
{
    uint8_t i;
    uint16_t address= eeprom_offset;
    for (i=0; i<num_bytes; i++)
    {
        dest[i] = EEPROM.read(address++);
    }
}

//Copy NWKSKEY from EEPROM to LMIC structure
void get_nwkkey_from_eeprom(void)
{
    bytes_eeprom2ram(LMIC.nwkKey, offsetof(NodeConfig, nwkkey), sizeof(LMIC.nwkKey));
}

void copy_appskey_from_eeprom(void)
{
    bytes_eeprom2ram(LMIC.artKey, offsetof(NodeConfig, appskey), sizeof(LMIC.artKey));
}
*/

uint8_t is_pcint_enabled(int pin)
{
    uint8_t pcintnum = digitalPinToPCINT(pin);
    uint8_t pcintMask = (1 << (pcintnum%8));
    uint8_t enabled;
    //Serial.print("Pin: "); Serial.print(pin);
    //Serial.print(", pcintport: "); Serial.print(pcintnum/8);
    //Serial.print(", pcintMask: 0x"); Serial.print(pcintMask,HEX);
    switch (pcintnum/8) // geht nicht anders weil die Mask Register nicht in aufeinanderfolgenden Adressen liegen.
    {
        case 0:
            enabled = PCMSK0 & pcintMask;
            break;
        case 1:
            enabled = PCMSK1 & pcintMask;
            break;
        case 2:
            enabled = PCMSK2 & pcintMask;
            break;
        case 3:
            enabled = PCMSK3 & pcintMask;
            break;
        default:
            return 0;
    }
    /*
    if (enabled)
        Serial.println(", enabled"); 
    else
        Serial.println(", disabled"); 
    */
    return enabled;
}

/*****************************************************************************/
/*   LMIC Frame counter  EEPROM Administration */
FrameCounter FrmCnt((sizeof(NodeConfig)/64)*64 +64);
/*****************************************************************************/

/*****************************************************************************/

uint16_t debounce_time[4];

int8_t debouncePin(int8_t PCIPin, uint16_t DEBOUNCE_TIME, uint16_t DEBOUNCE_MAX_TIME)
{
    unsigned long now, db_start, db_null;
    now = millis();
    db_start = now;
    db_null = now; 
    int8_t pressed = -1;

    int8_t state = digitalRead(PCIPin);
    
    while (now - db_null < DEBOUNCE_MAX_TIME )
    {
        state = digitalRead(PCIPin);
        if (digitalRead(PCIPin) != state)
        {
            state= !state;
            if ((now-db_null)> (DEBOUNCE_MAX_TIME-DEBOUNCE_TIME))// cannot fulfill debounce condition anymore -> return
            {
                break;
            }
            db_start = now; // restart counting, Bounce happened.
        }
        else if (now - db_start > DEBOUNCE_TIME)
        {
            pressed=  state;
            break;
        }
        
        now = millis();
    }

    return pressed;  
}


bool on_pci(uint8_t event)
{
    uint8_t trigger;
    int8_t  PCIPin;
     uint8_t enabled;
    uint8_t *pnode = (uint8_t*) &node;
    
    PCIPin = (int8_t) *(pnode + offsetof(NodeConfig, PCI0Pin) + event);
    trigger = *(pnode + offsetof(NodeConfig, PCI0Trig) + event) & 0x3;
    enabled = (pnode[0]>>(1+event)) & 0x1;
    
    /*
    #ifdef USE_SERIAL
    Serial.print("event: ");        Serial.print(event);
    Serial.print(", PCIPin: ");     Serial.print(PCIPin);
    Serial.print(", Trigger");      Serial.print(trigger);
    Serial.print(", is_enabled: "); Serial.print(enabled);
    Serial.println();
    #endif
    */
    
    if (enabled) 
    {
        disablePinChangeInterrupt(digitalPinToPinChangeInterrupt(PCIPin));
        int8_t state = debouncePin(PCIPin, debounce_time[event], debounce_time[event]+50);
        if ((state == 0 && trigger==FALLING) || (state==1 && trigger == RISING) || (state !=-1 && trigger == CHANGE))
        {
            //action:
            if (node.Flag_LED_enable) digitalWrite(node.LedPin, HIGH);
            led_on_time = os_getTime();
            #ifdef USE_SERIAL
            Serial.print("PCI");Serial.println(event);Serial.flush();
            #endif
            return true;
        }
    }
    if (debounce_time[event] != 0)
        // re-enable PCI to allow new triggering immediately
        // do not re-enable PCI in case Debouncing / multiple triggering per time is handled
        // by the default handler - 8s retrigger hold off
        enablePinChangeInterrupt(digitalPinToPinChangeInterrupt(PCIPin));
    return false;
}


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
        if (node.Flag_use_crystal_timer)
        {
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
        }
        
        else // low power RC Oscillator
        {
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
        }
        //*********** after wake up...   *****/

        if (!event_triggered)
        {
            watchdog_counter++;
            
            if (node.Flag_PCI0_enable && !is_pcint_enabled(node.PCI0Pin))
            {
                enablePinChangeInterrupt(digitalPinToPinChangeInterrupt(node.PCI0Pin));
                Serial.println("PCI0 enabled");
            }
            
            pir_inhibit_counter++;
            /*
            Serial.print("pir inh.count: "); Serial.println(pir_inhibit_counter);
            Serial.print("pir_is_off: "); Serial.print(pir_is_off);
            Serial.print(", Flag_PIR_enable: "); Serial.print(node.Flag_PIR_enable);
            Serial.print(", pir_inhibit_expired: "); Serial.print(pir_inhibit_expired);
            Serial.print(", status of PIRVccPin: "); Serial.print(digitalRead(node.PIRVccPin));
            Serial.println();
            Serial.flush();
            */
            
            if (node.Flag_PCI2_enable && !is_pcint_enabled(node.PCI2Pin)) 
            {
                enablePinChangeInterrupt(digitalPinToPinChangeInterrupt(node.PCI2Pin));
                Serial.println("PCI2 enabled");
            }
            
            if (node.Flag_PCI3_enable && !is_pcint_enabled(node.PCI3Pin))
            {
                enablePinChangeInterrupt(digitalPinToPinChangeInterrupt(node.PCI3Pin));
                Serial.println("PCI3 enabled");
            }
            Serial.flush();
        


            // switch ON PIR after dead time
            pir_inhibit_expired = (pir_inhibit_counter >= node.PIRDisableDelay) && pir_is_off && node.Flag_PIR_enable;
            if (pir_inhibit_expired) // Totzeit fuer PIR ist abgelaufen
            {
                //pir_inhibit_counter = 0;
                pir_is_off = false;
                digitalWrite(node.PIRVccPin, HIGH);
                enablePinChangeInterrupt(digitalPinToPinChangeInterrupt(node.PIRPin));
                /*
                no need to wait for panasonic PIR, because its always on.
                // need to wait for 2.5s because data pin goes high for 2.5s when AM312PIR ist switched ON
                LowPower.powerDown(SLEEP_500MS , ADC_OFF, BOD_OFF);
                LowPower.powerDown(SLEEP_2S, ADC_OFF, BOD_OFF);
                */
                #ifdef USE_SERIAL
                Serial.println("PIR aktiv."); Serial.flush();
                #endif
                event_triggered=0;
                continue;
            }
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
                long Vcal_x_ADCcal = (long)node.AdcCalValue * node.VccAtCalmV;
                msg.vcc = Vcal_x_ADCcal / vcc_reading;
                msg.count = LMIC.seqnoUp;
                msg.brightness = brightness_with_LDR(node.LedPin_K);
                #ifdef USE_HTU21D
                Measure_HTU21D(msg);
                #elif defined USE_BME280
                Measure_BME280(msg);
                #elif defined USE_DS18B20
                Measure_DS18B20(msg);
                #endif
                do_send((uint8_t*)&msg, sizeof(msg));
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
            if (event_triggered & 0x1) // PCI0 event
            {
                if (!on_pci(0))
                {
                    event_triggered &= 0xFE; //0b 1111 1110   // clear event: we don't send a packet here
                    continue;
                }
            }
            
            if (event_triggered & 0x2) //PCI1 reserved for PIR sensor
            {
                if (pir_is_off || !node.Flag_PIR_enable)
                {
                    event_triggered  &= 0xFD; // 0b1111 1101
                    continue; // triggered during dead time
                }
                else
                {
                    disablePinChangeInterrupt(digitalPinToPinChangeInterrupt(node.PIRPin));
                    if (node.Flag_LED_enable) digitalWrite(node.LedPin, HIGH);
                    led_on_time = os_getTime();
                    #ifdef USE_SERIAL
                    Serial.println("PIR");Serial.flush();
                    #endif
                    pir_inhibit_counter = 0;
                    pir_inhibit_expired = false;
                    pir_is_off = true;
                    //digitalWrite(node.PIRVccPin, LOW); // uncomment for AM312 PIR
                }
                /*
                Serial.print("pir_is_off: "); Serial.print(pir_is_off);
                Serial.print(", Flag_PIR_enable: "); Serial.print(node.Flag_PIR_enable);
                Serial.print(", pir_inhibit_expired: "); Serial.print(pir_inhibit_expired);
                Serial.print(", status of PIRVccPin: "); Serial.print(digitalRead(node.PIRVccPin));
                Serial.println();
                */
            }
            if (event_triggered & 0x4) // PCI2 Event
            {
                if (!on_pci(2))
                {
                    // false alarm
                    event_triggered &= 0xFB; //0b 1111 1011
                    continue;
                }
            }
            if (event_triggered & 0x8) // PCI3 Event
            {
                if (!on_pci(3))
                {
                    event_triggered &= 0xF7; //0b 1111 0111
                    continue;
                }
            }


            uint8_t *pnode = (uint8_t*) &node;
            msg.event = (event_triggered &0x0F) << 1;
            event_triggered =0;
            //Serial.print("msg.event: 0x"); Serial.println(msg.event,HEX); Serial.flush();
            //Serial.print("pnode[0] : 0x"); Serial.println(pnode[0],HEX); Serial.flush();
            //Serial.print("LMIC.seqnoUp = "); Serial.println(LMIC.seqnoUp); Serial.flush();
            if (pnode[0] & msg.event)
            {
                long Vcal_x_ADCcal = (long)node.AdcCalValue * node.VccAtCalmV;
                msg.vcc = Vcal_x_ADCcal / vcc_reading;
                msg.count = LMIC.seqnoUp;
                msg.brightness = brightness_with_LDR(node.LedPin_K);
                #ifdef USE_HTU21D
                Measure_HTU21D(msg);
                #elif defined USE_BME280
                Measure_BME280(msg);
                #elif defined USE_DS18B20
                Measure_DS18B20(msg);
                #endif

                do_send((uint8_t*)&msg, sizeof(msg));
                return;
            }
            else
            {
                digitalWrite(node.LedPin, LOW);
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
                        pnode[0] = (pnode[0] &0xC0) | (data[i] & 0x3F);
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
            pNode += data[0]&0x3f; // offset into configuration structure
            for (i=1; i<LMIC.dataLen; i++)
            {
                *pNode++ = data[i]; // copy all data beginning at <offset>
            }
            if (data[0] & 0x40) // eeprom update bit is set
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
            
            FrmCnt.set_frame_counter(LMIC.seqnoUp);
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


void do_send(uint8_t* data, uint8_t datalen)
{
    Serial.print("LMIC.seqnoUp = "); Serial.println(LMIC.seqnoUp); Serial.flush();
    // Check if there is not a current TX/RX job running
    if (LMIC.opmode & OP_TXRXPEND)
    {
        #ifdef USE_SERIAL
        Serial.println(F("OP_TXRXPEND, not sending"));
        #endif
    } 
    else 
    {
        // Prepare upstream data transmission at the next possible time.
        LMIC_setTxData2(1, data, datalen, 0);
        #ifdef USE_SERIAL
        Serial.println(F("Packet queued"));
        #endif
    }
    // Next TX is scheduled after TX_COMPLETE event.
}

/***********************   LMIC SETUP ***********************/

// Pin mapping
// By default, radio interrupts are not used.
// change lmic_project_config.h if interrupts are to be used. 

const lmic_pinmap lmic_pins = {
    .nss = RFM95_CS,                       // chip select on TiLLi
    .rxtx = LMIC_UNUSED_PIN,
    .rst = LMIC_UNUSED_PIN,          // reset pin
    .dio = {RFM95_DIO0, RFM95_DIO1, RFM95_DIO2},  //DIO0 und DIO1 on TiLLi
};



// These callbacks are only used in over-the-air activation, so they are
// left empty here (we cannot leave them out completely unless
// DISABLE_JOIN is set in arduino-lmic/project_config/lmic_project_config.h,
// otherwise the linker will complain).
void os_getArtEui (u1_t* buf) { (void) buf;}
void os_getDevEui (u1_t* buf) { (void) buf;}
void os_getDevKey (u1_t* buf) { (void) buf;}





void setup() {
    #ifdef USE_SERIAL
    while (!Serial); // wait for Serial to be initialized
    Serial.begin(SERIALBAUD);
    #endif
    delay(100);
    #ifdef USE_SERIAL
    Serial.println(F("Start"));
    Serial.println(F(DEVICE_NAME));
    #endif

    /***                     ***/
    /***     CALIBRATE?      ***/
    /***                     ***/
    CalMode.configure();
    

    pinMode(node.LedPin, OUTPUT);


    if (node.Flag_use_crystal_timer)
    {
        #ifdef USE_SERIAL
        Serial.println("crystal timer");
        #endif
        setup_timer2(T_8SECONDS);
    }
    else
    {
        #ifdef USE_SERIAL
        Serial.println("Using WDT");
        #endif
    }

    /*
    Serial.print("Flag_heartbeat_enable "); Serial.println(node.Flag_heartbeat_enable);
    Serial.print("Flag_PCI0_enable "); Serial.println(node.Flag_PCI0_enable);
    Serial.print("Flag_PIR_enable "); Serial.println(node.Flag_PIR_enable);
    Serial.print("Flag_PCI2_enable "); Serial.println(node.Flag_PCI2_enable);
    Serial.print("Flag_PCI3_enable "); Serial.println(node.Flag_PCI3_enable);
    Serial.print("Flag_LED_enable "); Serial.println(node.Flag_LED_enable);
    Serial.print("Flag_LED_blink_active "); Serial.println(node.Flag_LED_blink_active);
    Serial.print("Flag_use_crystal_timer "); Serial.println(node.Flag_use_crystal_timer);
    Serial.print("Flag_reserved "); Serial.println(node.Flag_reserved);
    */
    /*
    Serial.print("LDR Pin: "); Serial.println(node.LedPin_K);
    Serial.print("SDA PIn: "); Serial.println(SDAPIN);
    Serial.print("SCL Pin: "); Serial.println(SCLPIN);
    */
    //uint8_t *pnode = (uint8_t*) &node;
    //Serial.print("Flags LSB "); Serial.println(pnode[0], HEX);
    //Serial.print("Flags LSB "); Serial.println(*((uint8_t*) &node), HEX);// ja man koennte das so machen, aber der Code ist unlesbar!
    //Serial.print("Flags MSB "); Serial.println(pnode[1], HEX);
    //Serial.print("LMIC size in RAM: "); Serial.println(sizeof(LMIC)); // LMIC measures 796 BYtes!


    if (node.PCI0Pin >=0) // attach if pin exists; disable if flag not set;
    {
        pinMode(node.PCI0Pin, node.PCI0Trig>>2);  // set the pin to input or input with Pullup
        attachPinChangeInterrupt(digitalPinToPinChangeInterrupt(node.PCI0Pin), wakeUp0, node.PCI0Trig&0x3);
        if (!node.Flag_PCI0_enable)
            disablePinChangeInterrupt(digitalPinToPinChangeInterrupt(node.PCI0Pin));
    }
    else
    {
        node.Flag_PCI0_enable =0;
    }

    if (node.PIRPin >=0  && node.PIRVccPin >=0) // THE PIR SENSOR, do not change these settings!
    {
        pinMode(node.PIRPin, INPUT);
        // turn PIR Sensor ON so that it works immediately after start up.
        // But do activate the interrupt AFTER this so that no event is triggered.
        pinMode(node.PIRVccPin, OUTPUT);
        digitalWrite(node.PIRVccPin, HIGH);
        pir_is_off = true;
        delay(100);
        attachPinChangeInterrupt(digitalPinToPinChangeInterrupt(node.PIRPin), wakeUp1, RISING);
        disablePinChangeInterrupt(digitalPinToPinChangeInterrupt(node.PIRPin));
    }
    else
    {
        node.Flag_PIR_enable =0;
    }

    if (node.PCI2Pin >=0)
    {
        pinMode(node.PCI2Pin, node.PCI2Trig>>2);  // set the pin to input or input with Pullup
        attachPinChangeInterrupt(digitalPinToPinChangeInterrupt(node.PCI2Pin), wakeUp2, node.PCI2Trig&0x3);
        if (!node.Flag_PCI2_enable)
            disablePinChangeInterrupt(digitalPinToPinChangeInterrupt(node.PCI2Pin));
    }
    else
    {
        node.Flag_PCI2_enable =0;
    }

    if (node.PCI3Pin >=0)
    {
        pinMode(node.PCI3Pin, node.PCI3Trig>>2);  // set the pin to input or input with Pullup
        attachPinChangeInterrupt(digitalPinToPinChangeInterrupt(node.PCI3Pin), wakeUp3, node.PCI3Trig&0x3);
        if (!node.Flag_PCI3_enable)
            disablePinChangeInterrupt(digitalPinToPinChangeInterrupt(node.PCI3Pin));
    }
    else
    {
        node.Flag_PCI3_enable =0;
    }

    // define debounce time frame - signal must be stable for debounce_time milliseconds before any action;
    // if state changes within this time, interrupt is discarded. 
    debounce_time[0] = PCI0_DEBOUNCE_TIME;
    debounce_time[1] = PCI1_DEBOUNCE_TIME;
    debounce_time[2] = PCI2_DEBOUNCE_TIME;
    debounce_time[3] = PCI3_DEBOUNCE_TIME;
    
    
    
    
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


    /***********************   LMIC Initialization ***********************/
    os_init();
    // Reset the MAC state. Session and pending data transfers will be discarded.
    LMIC_reset();
    // Set static session parameters. Instead of dynamically establishing a session
    // by joining the network, precomputed session parameters are be provided.
   
    LMIC.nwkKey = node.nwkkey;
    LMIC.artKey = node.appskey;
    LMIC_setSession (0x13, node.devaddr, NULL, NULL);
    

    
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

    // accept 2% absolute clock error
    LMIC_setClockError(MAX_CLOCK_ERROR * 2 / 100);
    
    FrmCnt.begin();
    LMIC.seqnoUp = FrmCnt.get_frame_counter();
    
    // Start job
    Payload msg;
    msg.event =0; //  0 = Restart Flag
    msg.count= LMIC.seqnoUp;
    msg.vcc = readVcc(); // value to be used for calibration
    msg.brightness = brightness_with_LDR(node.LedPin_K);
    #ifdef USE_BME280
    Measure_BME280(msg);
    #elif defined USE_HTU21D
    Measure_HTU21D(msg);
    #elif defined USE_DS18B20
    Measure_DS18B20(msg);
    #endif

    #ifdef USE_SERIAL
    Serial.flush();
    #endif

    #ifndef TESTLOOP
        do_send((uint8_t*)&msg, sizeof(msg));
    #endif
}

#ifndef TESTLOOP
void loop() {
    os_runloop_once();
    if (os_getTime() - led_on_time > 6250) //100 ms = 6250 * 16us
    {
        digitalWrite(node.LedPin, LOW);
    }
    if( tx_is_complete )
    {
        tx_is_complete = false;
        wake_from_sleep_service_routine();
    }
}
#endif


#if defined TESTLOOP
void loop()
{
    Payload msg;
    static bool time2setup_complete=false;
    Serial.flush();
    if (node.Flag_use_crystal_timer)
    {
        if (!time2setup_complete)
        {
            setup_timer2(T_8SECONDS);
            time2setup_complete = true;
        }    
        LowPower.powerSave(SLEEP_FOREVER, ADC_OFF, BOD_OFF, TIMER2_ON);
    }
    else
        LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);
    #ifdef USE_BME280
    Measure_BME280(msg);
    #elif defined USE_HTU21D
    Measure_HTU21D(msg);
    #elif defined USE_DS18B20
    Measure_DS18B20(msg);
    #endif
}
#endif