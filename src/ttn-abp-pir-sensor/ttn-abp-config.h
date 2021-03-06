/*
This configuration file contains 4 typical configurations:
TINO1: PIR + HTU21D (motion Sensor + temperature + humidity )
TINO2: PIR  (motion sensor only)
TINO3: PIR + BME280 (motion Sensor + temperature + humidity + barometric pressure)
TINO4: PIR + DS18B20 Temperature sensor(s)
PIR can be disabled by setting node.Flag_PIR_enable=0;
Due to limited flash space, sensors are mutually exclusive. 
*/

#define TINO1
//#define TINO2
//#define TINO3
//#define TINO4

#ifdef TINO1
// LoRaWAN NwkSKey, network session key
// This should be in big-endian (aka msb).
static const PROGMEM u1_t NWKSKEY[16] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };

// LoRaWAN AppSKey, application session key
// This should also be in big-endian (aka msb).
static const u1_t PROGMEM APPSKEY[16] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };

// LoRaWAN end-device address (DevAddr)
// See http://thethingsnetwork.org/wiki/AddressSpace
// The library converts the address to network byte order as needed, so this should be in big-endian (aka msb) too.
static const u4_t DEVADDR = 0x0; // <-- Change this address for every node!


// calibrated reference value for correct VCC calculations
// this is: VCC(calibration_time] * DAC_VALUE[calibration_time]
// DAC value is exposed at "restart" event in the VCC Field.
#define REFERENCE_VALUE 1112100

// Schedule TX every this many seconds (might become longer due to duty
// cycle limitations).
#define TX_INTERVAL 1800

// select which sensor is used
//#define USE_BME280
#define USE_HTU21D
//#define USE_DS18B20


#elif defined TINO2
static const PROGMEM u1_t NWKSKEY[16] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
static const u1_t PROGMEM APPSKEY[16] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
static const u4_t DEVADDR = 0x0;

#define REFERENCE_VALUE 1138500
#define TX_INTERVAL 3280

//#define USE_BME280
//#define USE_HTU21D
//#define USE_DS18B20



#elif defined TINO3 
//for Testing of BME280
static const PROGMEM u1_t NWKSKEY[16] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
static const u1_t PROGMEM APPSKEY[16] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
static const u4_t DEVADDR = 0x0;

#define REFERENCE_VALUE 1135200
#define TX_INTERVAL 1680

#define USE_BME280
//#define USE_HTU21D
//#define USE_DS18B20



#elif defined TINO4
//for Testing of BME280
static const PROGMEM u1_t NWKSKEY[16] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
static const u1_t PROGMEM APPSKEY[16] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
static const u4_t DEVADDR = 0x0;

#define REFERENCE_VALUE 1135200
#define TX_INTERVAL 1680

//#define USE_BME280
//#define USE_HTU21D
#define USE_DS18B20
#endif




// Schedule TX every this many seconds (might become longer due to duty
// cycle limitations).


/********do not edit!  ********/
#define SENDDELAY TX_INTERVAL/8
/*****************************/

//Dead time of PIR Sensor after it has triggered, to save Battery and to make sure to comply with regulations.
// Theoretically the PIR Sensor can trigger every 2.3 Seconds. 
// for SF7BW125 transmit time is approx. 50ms. Fair use policy asks for not more than 30s
// total transmit time per 24h. This makes 600 Packets per day, so they should be at least 144s
// apart. 144/8 = 18
#define DEAD_TIME 24

/********do not edit!  ********/
#define PIR_INHIBIT_DELAY DEAD_TIME/8
/*****************************/

// Transmit Power in dBm
#define TXPOWER 2

// Coding and Modulation scheme
#define DATARATE DR_SF7

//Baud Rate of serial interface.
//recommended 4800 Bd for 1MHz Clock, 19200Bd for 8MHz clock
//hardcoded, not configurable at runtime.
#define SERIALBAUD 4800

// Pins of LED
#define LED_PIN A0
#define LED_PIN_K A1 // Kathode if -1 it is assumed the Kathode is on GND
// Pin where the VCC of the pIR is connected
#define PIRVCCPIN 6

//Pin where the Data output of the PIR Sensor is connected
#define PCI1PIN 5

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


#define PCI0PIN 7
#define PCI0PIN_MODE INPUT_PULLUP
#define PCI0PIN_TRIGGER FALLING

#define PCI2PIN -1
#define PCI2PIN_MODE INPUT_PULLUP
#define PCI2PIN_TRIGGER FALLING

#define PCI3PIN -1
#define PCI3PIN_MODE INPUT_PULLUP
#define PCI3PIN_TRIGGER FALLING

// RFM95 Pins 
// do not change, these Pins are TiNo hardware specific!
#define RFM95_CS 10 // chip select
#define RFM95_DIO0 2 // Dio0
#define RFM95_DIO1 3 //Dio1
#define RFM95_DIO2 LMIC_UNUSED_PIN //Dio1

// I2C Bus
//#define USE_BME280
//#define USE_HTU21D

#ifdef USE_HTU21D
#define SDAPIN A4 //18
#define SCLPIN A5  //19 

#elif defined USE_BME280
#define SDAPIN A5 //19
#define SCLPIN A4  //18
#endif
#define I2CPOWERPIN 9

/**** END OF USER DEFINABLE PRESETS ****/
unsigned int readVcc(void);

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
    uint16_t Flag_reserved:9;
    uint8_t  DataRate = DATARATE;   // LoRa SF, BW and FEC scheme
    uint8_t  TxPower = TXPOWER;     // Transmit Power in dBm
    uint16_t SendDelay = SENDDELAY;  // Interval between Heartbeats
    uint16_t PIRDisableDelay = PIR_INHIBIT_DELAY; // Time the PIR is disabled after it has triggered
    int8_t   LedPin = LED_PIN;  // Pins are signed values, -1 indicates "unused"
    int8_t   LedPin_K = LED_PIN_K;
    int8_t   PIRVccPin = PIRVCCPIN;
    //int8_t   SDAPin = SDAPIN;  // TiNo-HP and TiNo-LC
    //int8_t   SCLPin = SCLPIN;  // TiNo-HP and TiNo-LC
    int8_t   I2CPowerPin = I2CPOWERPIN;
    int8_t   PCI0Pin = PCI0PIN;    
    int8_t   PIRPin =  PCI1PIN;  // PCI1Pin is renamed PIRPin
    int8_t   PCI2Pin = PCI2PIN;
    int8_t   PCI3Pin = PCI3PIN;
    uint8_t  PCI0Trig = (PCI0PIN_MODE<<2) | PCI0PIN_TRIGGER;
    uint8_t  PIRTrig = (INPUT<<2) | RISING;
    uint8_t  PCI2Trig = (PCI2PIN_MODE<<2) | PCI2PIN_TRIGGER;
    uint8_t  PCI3Trig = (PCI3PIN_MODE<<2) | PCI3PIN_TRIGGER;
    uint32_t VccReference = REFERENCE_VALUE;
    uint16_t checksum = 0x296E;
    
    NodeConfig()
    {
        Flag_heartbeat_enable=1; //0
        Flag_PCI0_enable=1;      //1
        Flag_PIR_enable=1;       //2
        Flag_PCI2_enable=0;      //3
        Flag_PCI3_enable=0;      //4
        Flag_LED_enable=1;       //5
        Flag_LED_blink_active=0; //6
        Flag_reserved =0;        // 7- 15
    }
};

NodeConfig node;
