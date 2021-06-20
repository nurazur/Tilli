/*
This configuration file contains typical configurations
TILLI0: PIR + HTU21D (motion Sensor + temperature + humidity + vibration sensor + push button)
*/

#ifndef NODE_CONFIGURATION
#define NODE_CONFIGURATION

#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>


/******* do not edit!  ********/
#define SENDDELAY TX_INTERVAL/8
/******************************/
/******* do not edit!  ********/
#define PIR_INHIBIT_DELAY DEAD_TIME/8
/******************************/

/******* do not edit!  ********/
#define USE_TTN_STACK_V3 5
#define USE_TTN_STACK_V2 1
/******************************/


#define TILLI_2
#define TTN_STACK_VERSION

#define HARDWARE_TILLI_BOLINK_V0_2
//#define HARDWARE_MEGABRICK_DEV_BOARD
//#define HARDWARE_TILLI_BOLINK_V1_1_0



#if defined TILLI_0 //same as Tino2
    #define DEVICE_NAME "Tilli 0 (aka Tino2)"
    #define REFERENCE_ADC_VALUE 344
    #define TX_INTERVAL 1800  // 30 min.
    #define PCI0_DEBOUNCE_TIME 0
    #define PCI1_DEBOUNCE_TIME 0
    #define PCI2_DEBOUNCE_TIME 0
    #define PCI3_DEBOUNCE_TIME 0
    #define TTN_STACK_VERSION USE_TTN_STACK_V2


#elif defined TILLI_1
    #define DEVICE_NAME "Tilli 1"
    #define REFERENCE_ADC_VALUE 340 // cald.
    #define TX_INTERVAL 1800  // 30 min.
    #define PCI0_DEBOUNCE_TIME 1000
    #define PCI1_DEBOUNCE_TIME 0
    #define PCI2_DEBOUNCE_TIME 0
    #define PCI3_DEBOUNCE_TIME 100
    #define TTN_STACK_VERSION USE_TTN_STACK_V2
    
#elif defined TILLI_2
    #define DEVICE_NAME "Tilli 2"
    #define REFERENCE_ADC_VALUE 329
    #define TX_INTERVAL 600  // 10 min.
    #define PCI0_DEBOUNCE_TIME 0
    #define PCI1_DEBOUNCE_TIME 0
    #define PCI2_DEBOUNCE_TIME 0
    #define PCI3_DEBOUNCE_TIME 0
    #define TTN_STACK_VERSION USE_TTN_STACK_V3
    
#elif defined TILLI_3
    #define DEVICE_NAME "Tilli 3"
    #define REFERENCE_ADC_VALUE 337
    #define TX_INTERVAL 600  // 10 min.
    #define PCI0_DEBOUNCE_TIME 0
    #define PCI1_DEBOUNCE_TIME 0
    #define PCI2_DEBOUNCE_TIME 0
    #define PCI3_DEBOUNCE_TIME 0
    #define TTN_STACK_VERSION USE_TTN_STACK_V2
    
#elif defined TILLI_4
    #define DEVICE_NAME "Tilli 4" // Tilli 4 has the same NWKSKEY and APPSKEY as Tilli1, only DEVADDR is different!
    #define REFERENCE_ADC_VALUE 339
    #define TX_INTERVAL 1800  // 30 min.
    #define PCI0_DEBOUNCE_TIME 0
    #define PCI1_DEBOUNCE_TIME 0
    #define PCI2_DEBOUNCE_TIME 0
    #define PCI3_DEBOUNCE_TIME 0
    #define USE_HTU21D_INVERTED_I2C // Pinout of HTU21D Modules is inverted from usual + - CL DA 
    #define USE_I2C_PULLUPS true
    #define TTN_STACK_VERSION USE_TTN_STACK_V2
    
#elif defined TILLI_5
    #define DEVICE_NAME "Tilli 5"
    #define REFERENCE_ADC_VALUE 339
    #define TX_INTERVAL 600  // 10 min.
    #define PCI0_DEBOUNCE_TIME 0
    #define PCI1_DEBOUNCE_TIME 0
    #define PCI2_DEBOUNCE_TIME 0
    #define PCI3_DEBOUNCE_TIME 0
    #define TTN_STACK_VERSION USE_TTN_STACK_V2
#endif


#if defined HARDWARE_TILLI_BOLINK_V0_2

    // Tilli_Bolink devices have HTU21D/SH21 chip
    #ifndef USE_HTU21D_INVERTED_I2C
    #define USE_HTU21D
    #endif
    
    //#define USE_BME280
    #define USE_MAX31865
    
    // Tilli_Bolink devices have a 32.768 Crystal by default
    // set to 0 to use built-in RC oscillator timer
    #define USE_CRYSTAL_TIMER 1

    //Dead time of PIR Sensor after it has triggered, to save Battery and to make sure to comply with regulations.
    // Theoretically the PIR Sensor can trigger every 2.3 Seconds. 
    // for SF7BW125 transmit time is approx. 50ms. Fair use policy asks for not more than 30s
    // total transmit time per 24h. This makes 600 Packets per day, so they should be at least 144s
    // apart. 144/8 = 18
    #define DEAD_TIME 144


    // Transmit Power in dBm
    #define TXPOWER 2

    // Coding and Modulation scheme
    // possible data rates in Europe:
    //  DR_SF12
    //  DR_SF11 
    //  DR_SF10
    //  DR_SF9 
    //  DR_SF8
    //  DR_SF7  (BW125)
    //  DR_SF7B (BW250)
    //  DR_FSK
    #define DATARATE DR_SF7

    //Baud Rate of serial interface.
    //recommended 4800 Bd for 1MHz Clock, 19200Bd for 8MHz clock
    //hardcoded, not configurable at runtime.
    #define SERIALBAUD 4800

    // Pins of LED
    #define LED_PIN 1
    #define LDR_PIN 28 // Kathode: if -1 it is assumed the Kathode is on GND



    //Pin Change Interrupts 

    /*
    PCIxTrigger bits 0 and 1:
    0bxx00 LOW
    0bxx01 CHANGE
    0bxx10 FALLING (normal use case)
    0bxx11 RISING

    PCIxTrigger bits 2 and 3:
    0b00xx INPUT
    0b01xx OUTPUT
    0b10xx INPUT_PULLUP (normal use case)

    PCIxTrigger bits 4...7:
    reserved. Set to 0. 
    0b0000xxxx
    */


    #define PCI0PIN 20 // push button
    #define PCI0PIN_MODE INPUT_PULLUP
    #define PCI0PIN_TRIGGER FALLING

    // Pin where the VCC of the PIR is connected
    #define PIRVCCPIN A5
    //Pin where the Data output of the PIR Sensor is connected
    #define PCI1PIN A6

    #define PCI2PIN 21 //Vibration Sensor
    #define PCI2PIN_MODE INPUT_PULLUP
    #define PCI2PIN_TRIGGER FALLING

    #define PCI3PIN -1
    #define PCI3PIN_MODE INPUT_PULLUP
    #define PCI3PIN_TRIGGER FALLING

    // RFM95 Pins 
    // do not change, these Pins are hardware specific!
    #define RFM95_CS 4 // chip select
    #define RFM95_DIO0 10 // Dio0 --> INT0
    #define RFM95_DIO1 11 //Dio1 --> INT1
    #define RFM95_DIO2 LMIC_UNUSED_PIN

    #ifdef USE_HTU21D_INVERTED_I2C
    #define SDAPIN 16
    #define SCLPIN 17
    
    #elif defined USE_HTU21D
    #define SDAPIN 17
    #define SCLPIN 16

    #elif defined USE_BME280
    #define SDAPIN 17
    #define SCLPIN 16

    #elif defined USE_DS18B20
    #define SDAPIN 17
    #endif
    #define I2CPOWERPIN 15

    #ifdef USE_MAX31865
    #define PT100
    #define RTD_CS_PIN  13
    #define RTD_PWR_PIN 19
    #define sMOSI 18
    #define sMISO 12
    #define sSCK  14
    
    

    #define FAULT_HIGH_THRESHOLD  0x9304  /* +350C */
    #define FAULT_LOW_THRESHOLD   0x2690  /* -100C */
    #endif
// end HARDWARE_TILLI_BOLINK_V0_2

#elif defined HARDWARE_TILLI_BOLINK_V1_1_0
// Tilli_Bolink devices have HTU21D/SH21 chip
    #ifndef USE_HTU21D_INVERTED_I2C
    #define USE_HTU21D
    #endif
    
    //#define USE_BME280
    #define USE_MAX31865
    
    // Tilli_Bolink devices have a 32.768 Crystal by default
    // set to 0 to use built-in RC oscillator timer
    #define USE_CRYSTAL_TIMER 1

    //Dead time of PIR Sensor after it has triggered, to save Battery and to make sure to comply with regulations.
    // Theoretically the PIR Sensor can trigger every 2.3 Seconds. 
    // for SF7BW125 transmit time is approx. 50ms. Fair use policy asks for not more than 30s
    // total transmit time per 24h. This makes 600 Packets per day, so they should be at least 144s
    // apart. 144/8 = 18
    #define DEAD_TIME 144


    // Transmit Power in dBm
    #define TXPOWER 2

    // Coding and Modulation scheme
    // possible data rates in Europe:
    //  DR_SF12
    //  DR_SF11 
    //  DR_SF10
    //  DR_SF9 
    //  DR_SF8
    //  DR_SF7  (BW125)
    //  DR_SF7B (BW250)
    //  DR_FSK
    #define DATARATE DR_SF7

    //Baud Rate of serial interface.
    //recommended 4800 Bd for 1MHz Clock, 19200Bd for 8MHz clock
    //hardcoded, not configurable at runtime.
    #define SERIALBAUD 4800

    // Pins of LED
    #define LED_PIN 1
    #define LDR_PIN 28 // Kathode if -1 it is assumed the Kathode is on GND



    //Pin Change Interrupts 

    /*
    PCIxTrigger bits 0 and 1:
    0bxx00 LOW
    0bxx01 CHANGE
    0bxx10 FALLING (normal use case)
    0bxx11 RISING

    PCIxTrigger bits 2 and 3:
    0b00xx INPUT
    0b01xx OUTPUT
    0b10xx INPUT_PULLUP (normal use case)

    PCIxTrigger bits 4...7:
    reserved. Set to 0. 
    0b0000xxxx
    */


    #define PCI0PIN 12 // push button
    #define PCI0PIN_MODE INPUT_PULLUP
    #define PCI0PIN_TRIGGER FALLING

    // Pin where the VCC of the PIR is connected
    #define PIRVCCPIN A6
    //Pin where the Data output of the PIR Sensor is connected
    #define PCI1PIN A5

    #define PCI2PIN 18 //Vibration Sensor
    #define PCI2PIN_MODE INPUT
    #define PCI2PIN_TRIGGER FALLING

    #define PCI3PIN -1
    #define PCI3PIN_MODE INPUT_PULLUP
    #define PCI3PIN_TRIGGER FALLING

    // RFM95 Pins 
    // do not change, these Pins are hardware specific!
    #define RFM95_CS 4 // chip select
    #define RFM95_DIO0 10 // Dio0 --> INT0
    #define RFM95_DIO1 11 //Dio1 --> INT1
    #define RFM95_DIO2 LMIC_UNUSED_PIN

    #ifdef USE_HTU21D_INVERTED_I2C
    #define SDAPIN 16
    #define SCLPIN 17
    
    #elif defined USE_HTU21D
    #define SDAPIN 17
    #define SCLPIN 16

    #elif defined USE_BME280
    #define SDAPIN 17
    #define SCLPIN 16

    #elif defined USE_DS18B20
    #define SDAPIN 17
    #endif
    #define I2CPOWERPIN 15
    
    #ifdef USE_MAX31865
    #define PT100
    #define RTD_CS_PIN  A0
    #define RTD_PWR_PIN A1

    #define FAULT_HIGH_THRESHOLD  0x9304  /* +350C */
    #define FAULT_LOW_THRESHOLD   0x2690  /* -100C */
    #endif
// end #define HARDWARE_TILLI_BOLINK_V1_1_0


#elif defined HARDWARE_MEGABRICK_DEV_BOARD

    //#define USE_BME280
    #define USE_HTU21D
    //#define USE_DS18B20
    
    // set USE_CRYSTAL_TIMER 1 if timer with 32.768 kHz Crytal shall be used. 
    //otherwise set to 0 
    #define USE_CRYSTAL_TIMER 1

    //Dead time of PIR Sensor after it has triggered, to save Battery and to make sure to comply with regulations.
    // Theoretically the PIR Sensor can trigger every 2.3 Seconds. 
    // for SF7BW125 transmit time is approx. 50ms. Fair use policy asks for not more than 30s
    // total transmit time per 24h. This makes 600 Packets per day, so they should be at least 144s
    // apart. 144/8 = 18
    #define DEAD_TIME 24

    // Transmit Power in dBm
    #define TXPOWER 2

    // Coding and Modulation scheme
    #define DATARATE DR_SF7

    //Baud Rate of serial interface.
    //recommended 4800 Bd for 1MHz Clock, 19200Bd for 8MHz clock
    //hardcoded, not configurable at runtime.
    #define SERIALBAUD 4800

    // Pins of LED
    #define LED_PIN 21
    #define LDR_PIN 28 // LDR Pin


    //Pin Change Interrupts 

    /*
    PCIxTrigger bits 0 and 1:
    0bxx00 LOW
    0bxx01 CHANGE
    0bxx10 FALLING (normal use case)
    0bxx11 RISING

    PCIxTrigger bits 2 and 3:
    0b00xx INPUT
    0b01xx OUTPUT
    0b10xx INPUT_PULLUP (normal use case)

    PCIxTrigger bits 4...7:
    reserved. Set to 0. 
    0b0000xxxx
    */


    #define PCI0PIN 19
    #define PCI0PIN_MODE INPUT_PULLUP
    #define PCI0PIN_TRIGGER FALLING

    // Pin where the VCC of the PIR is connected
    #define PIRVCCPIN 30 // A6
    //Pin where the Data output of the PIR Sensor is connected
    #define PCI1PIN 29 // A5

    #define PCI2PIN 20 // Vibration sensor
    #define PCI2PIN_MODE INPUT_PULLUP
    #define PCI2PIN_TRIGGER FALLING

    #define PCI3PIN -1
    #define PCI3PIN_MODE INPUT_PULLUP
    #define PCI3PIN_TRIGGER FALLING

    // RFM95 Pins 
    // do not change, these Pins are hardware specific!
    #define RFM95_CS 4 // chip select
    #define RFM95_DIO0 2 // hard wired on Mega Brick
    #define RFM95_DIO1 11
    #define RFM95_DIO2 LMIC_UNUSED_PIN // can be 10 

    #ifdef USE_HTU21D
    #define SDAPIN 17
    #define SCLPIN 16 

    #elif defined USE_BME280
    #define SDAPIN 16
    #define SCLPIN 17

    #elif defined USE_DS18B20
    #define SDAPIN 17
    #endif
    #define I2CPOWERPIN 18

#endif

/**** END OF USER DEFINABLE PRESETS ****/


class NodeConfig
{
    public:
    uint16_t Flag_heartbeat_enable:1;
    uint16_t Flag_PCI0_enable:1;
    uint16_t Flag_PIR_enable:1;
    uint16_t Flag_PCI2_enable:1;
    uint16_t Flag_PCI3_enable:1;
    uint16_t Flag_LED_enable:1;
    uint16_t Flag_LED_blink_active:1;
    uint16_t Flag_use_crystal_timer: 1;
    uint16_t Flag_reserved:8;
    uint8_t  DataRate = DATARATE;   // LoRa SF, BW and FEC scheme
    uint8_t  TxPower = TXPOWER;     // Transmit Power in dBm
    uint16_t SendDelay = SENDDELAY;  // Interval between Heartbeats
    uint16_t PIRDisableDelay = PIR_INHIBIT_DELAY; // Time the PIR is disabled after it has triggered
    int8_t   LedPin = LED_PIN;  // Pins are signed values, -1 indicates "unused"
    int8_t   LdrPin = LDR_PIN;
    int8_t   PIRVccPin = PIRVCCPIN;
    int8_t   I2CPowerPin = I2CPOWERPIN;
    int8_t   PCI0Pin = PCI0PIN;    
    int8_t   PIRPin =  PCI1PIN;  // PCI1Pin is renamed PIRPin
    int8_t   PCI2Pin = PCI2PIN;
    int8_t   PCI3Pin = PCI3PIN;
    uint8_t  PCI0Trig = (PCI0PIN_MODE<<2) | PCI0PIN_TRIGGER;
    uint8_t  PIRTrig = (INPUT<<2) | RISING;
    uint8_t  PCI2Trig = (PCI2PIN_MODE<<2) | PCI2PIN_TRIGGER;
    uint8_t  PCI3Trig = (PCI3PIN_MODE<<2) | PCI3PIN_TRIGGER;
    uint16_t AdcCalValue = REFERENCE_ADC_VALUE;
    uint16_t VccAtCalmV = 3300;
    uint8_t  nwkkey[16];
    uint8_t  appskey[16];
    uint32_t devaddr;
    uint16_t checksum = 0x296E;

    
    NodeConfig()
    {
        Flag_heartbeat_enable=1; //0
        Flag_PCI0_enable=1;      //1
        Flag_PIR_enable=0;       //2
        Flag_PCI2_enable=1;      //3
        Flag_PCI3_enable=0;      //4
        Flag_LED_enable=1;       //5
        Flag_LED_blink_active=0; //6
        Flag_use_crystal_timer= USE_CRYSTAL_TIMER; //7
        Flag_reserved =0;        // 8- 15
    }
};

static NodeConfig node;


#endif
