#include <SPI.h>
#include <SoftSPI.h>

// Create a new SPI port with:
// Pin 2 = MOSI,
// Pin 3 = MISO,
// Pin 4 = SCK

#define cs_pin A0
#define FAULT_HIGH_THRESHOLD  0x9304  /* +350C */
#define FAULT_LOW_THRESHOLD   0x2690  /* -100C */

//SoftSPI mySPI(4,5,6);  //mosi=4, miso=5, sck= 6
SoftSPI mysoftSPI(4,5,6);  //mosi=4, miso=5, sck= 6

/*
void writeRegister8(uint8_t r, uint8_t value)
{
  digitalWrite( cs_pin, LOW );
  mySPI.transfer( r | 0x80 );
  mySPI.transfer( value );
  digitalWrite( cs_pin, HIGH );
}


uint8_t readRegister8(uint8_t reg)
{
  uint8_t content;
  digitalWrite( cs_pin, LOW );
  mySPI.transfer( reg );
  content = mySPI.transfer(0x00);
  digitalWrite( cs_pin, HIGH );
  return content;
}



void setup() {
    pinMode(cs_pin, OUTPUT);
	mySPI.begin();
    //mySPI.setClockDivider( SPI_CLOCK_DIV2 );
    mySPI.setDataMode( SPI_MODE1 );
    //mySPI.setBitOrder(MSBFIRST);
	
    Serial.begin(9600);
}
*/


void writeRegister8(uint8_t r, uint8_t value)
{
  digitalWrite( cs_pin, LOW );
  mysoftSPI.transfer( r | 0x80 );
  mysoftSPI.transfer( value );
  digitalWrite( cs_pin, HIGH );
}


uint8_t readRegister8(uint8_t reg)
{
  uint8_t content;
  digitalWrite( cs_pin, LOW );
  mysoftSPI.transfer( reg );
  content = mysoftSPI.transfer(0x00);
  digitalWrite( cs_pin, HIGH );
  return content;
}

void setup() {
    pinMode(cs_pin, OUTPUT);
	mysoftSPI.begin();
    mysoftSPI.setClockDivider( SPI_CLOCK_DIV2 );
    mysoftSPI.setDataMode( SPI_MODE1 );
    mysoftSPI.setBitOrder(MSBFIRST);
	
    Serial.begin(9600);
}



void measure(SoftSPI &mySPI)
{
    uint8_t re;
    Serial.println("Sending value: 0x83 to Config register.");
    writeRegister8(0, 0x83);
    delay(10);
    re = readRegister8(0);
    Serial.print("Read value: 0x"); Serial.println(re,HEX);
    Serial.println();
    
    uint8_t thresh = FAULT_HIGH_THRESHOLD >>8;
    Serial.print("Sending high threshold MSB: "); Serial.println(thresh, HEX);
    Serial.print("Sending high threshold LSB: "); Serial.println((uint8_t)FAULT_HIGH_THRESHOLD, HEX);
    digitalWrite( cs_pin, LOW );
    mySPI.transfer( 0x83 );
    mySPI.transfer( thresh );
    mySPI.transfer( (uint8_t) FAULT_HIGH_THRESHOLD);
    digitalWrite( cs_pin, HIGH );
    delay(10);
    
    re = readRegister8(3);
    Serial.print("Read reg 3, value: 0x"); Serial.println(re,HEX);
    
    re = readRegister8(4);
    Serial.print("Read reg 4, value: 0x"); Serial.println(re,HEX);
    Serial.println();
    
    thresh = FAULT_LOW_THRESHOLD>>8;
    Serial.print("Sending low threshold MSB: "); Serial.println(thresh, HEX);
    Serial.print("Sending low threshold LSB: "); Serial.println((uint8_t)FAULT_LOW_THRESHOLD, HEX);
    digitalWrite( cs_pin, LOW );
    mySPI.transfer( 0x85 );
    mySPI.transfer( thresh );
    mySPI.transfer( (uint8_t) FAULT_LOW_THRESHOLD);
    digitalWrite( cs_pin, HIGH );
    delay(10);
    
    re = readRegister8(5);
    Serial.print("Read reg 5, value: 0x"); Serial.println(re,HEX);
    
    re = readRegister8(6);
    Serial.print("Read reg 6, value: 0x"); Serial.println(re,HEX);
    Serial.println();
}


void loop()
{
    measure(mysoftSPI);
    /*
    uint8_t re;
    Serial.println("Sending value: 0x83 to Config register.");
    writeRegister8(0, 0x83);
    delay(10);
    re = readRegister8(0);
    Serial.print("Read value: 0x"); Serial.println(re,HEX);
    Serial.println();
    
    uint8_t thresh = FAULT_HIGH_THRESHOLD >>8;
    Serial.print("Sending high threshold MSB: "); Serial.println(thresh, HEX);
    Serial.print("Sending high threshold LSB: "); Serial.println((uint8_t)FAULT_HIGH_THRESHOLD, HEX);
    digitalWrite( cs_pin, LOW );
    mySPI.transfer( 0x83 );
    mySPI.transfer( thresh );
    mySPI.transfer( (uint8_t) FAULT_HIGH_THRESHOLD);
    digitalWrite( cs_pin, HIGH );
    delay(10);
    
    re = readRegister8(3);
    Serial.print("Read reg 3, value: 0x"); Serial.println(re,HEX);
    
    re = readRegister8(4);
    Serial.print("Read reg 4, value: 0x"); Serial.println(re,HEX);
    Serial.println();
    
    thresh = FAULT_LOW_THRESHOLD>>8;
    Serial.print("Sending low threshold MSB: "); Serial.println(thresh, HEX);
    Serial.print("Sending low threshold LSB: "); Serial.println((uint8_t)FAULT_LOW_THRESHOLD, HEX);
    digitalWrite( cs_pin, LOW );
    mySPI.transfer( 0x85 );
    mySPI.transfer( thresh );
    mySPI.transfer( (uint8_t) FAULT_LOW_THRESHOLD);
    digitalWrite( cs_pin, HIGH );
    delay(10);
    
    re = readRegister8(5);
    Serial.print("Read reg 5, value: 0x"); Serial.println(re,HEX);
    
    re = readRegister8(6);
    Serial.print("Read reg 6, value: 0x"); Serial.println(re,HEX);
    Serial.println();
    */
    
    delay(2000);
}