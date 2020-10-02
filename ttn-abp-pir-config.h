#define TINO1
//#define TINO2
#ifdef TINO1

// LoRaWAN NwkSKey, network session key
// This should be in big-endian (aka msb).
static const PROGMEM u1_t NWKSKEY[16] = { 0xF0, 0x33, 0x0C, 0x97, 0xAF, 0x88, 0xF7, 0x0A, 0x87, 0x07, 0x83, 0xDD, 0x54, 0xE7, 0x2F, 0xA8 };

// LoRaWAN AppSKey, application session key
// This should also be in big-endian (aka msb).
static const u1_t PROGMEM APPSKEY[16] = { 0xB6, 0xA9, 0xAA, 0x97, 0x53, 0x9E, 0x77, 0x78, 0xDC, 0xB3, 0x7F, 0xF4, 0x56, 0x5E, 0x9D, 0x21 };

// LoRaWAN end-device address (DevAddr)
// See http://thethingsnetwork.org/wiki/AddressSpace
// The library converts the address to network byte order as needed, so this should be in big-endian (aka msb) too.
static const u4_t DEVADDR = 0x26011AB5; // <-- Change this address for every node!


// calibrated reference value for correct VCC calculations
// this is: VCC(calibration_time] * DAC_VALUE[calibration_time]
#define REFERENCE_VALUE 1112100

#endif


#ifdef TINO2
static const PROGMEM u1_t NWKSKEY[16] = { 0xE1, 0x65, 0x8E, 0xFE, 0x4F, 0x1D, 0x88, 0x71, 0x65, 0xA8, 0xF9, 0xF9, 0x21, 0x81, 0xB3, 0x89 };
static const u1_t PROGMEM APPSKEY[16] = { 0x79, 0x62, 0x9E, 0xAB, 0x89, 0x97, 0xDB, 0x90, 0x5C, 0xBD, 0x5A, 0x6B, 0x3E, 0x7E, 0x22, 0x0B };
static const u4_t DEVADDR = 0x26013484;

// calibrated reference value for correct VCC calculations
// this is: VCC(calibration_time] * DAC_VALUE[calibration_time]
#define REFERENCE_VALUE 1138500
#endif

// Schedule TX every this many seconds (might become longer due to duty
// cycle limitations).
#define TX_INTERVAL 3280

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
#define LED_PIN_K A1 // Kathode
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
#define SDAPIN A4
#define SCLPIN A5
#define I2CPOWERPIN 9



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
    uint16_t Flag_reserved:9;
    uint8_t  DataRate = DATARATE;   // LoRa SF, BW and FEC scheme
    uint8_t  TxPower = TXPOWER;     // Transmit Power in dBm
    uint16_t SendDelay = SENDDELAY;  // Interval between Heartbeats
    uint16_t PIRDisableDelay = PIR_INHIBIT_DELAY; // Time the PIR is disabled after it has triggered
    int8_t   LedPin = LED_PIN;  // Pins are signed values, -1 indicates "unused"
    int8_t   LedPin_K = LED_PIN_K;
    int8_t   PIRVccPin = PIRVCCPIN;
    int8_t   SDAPin = SDAPIN;  // TiNo-HP and TiNo-LC
    int8_t   SCLPin = SCLPIN;  // TiNo-HP and TiNo-LC
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
