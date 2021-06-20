// **********************************************************************************
// Copyright nurazur@gmail.com
// **********************************************************************************
// License
// **********************************************************************************
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.

// Licence can be viewed at
// http://www.fsf.org/licenses/gpl.txt

// Please maintain this license information along with authorship
// and copyright notices in any redistribution of this code
// *********************************************************************************

/***

Functions for interactive calibration and eeprom access over serial port

***/

extern unsigned int readVcc();


#include <EEPROM.h>

//#include "ttn-abp-1284-config.h"
#include "tilli1_1_1.h"
#include "mycodec.h"
#include "framecounter.h"
#include "calibrate.h"


Calibration::Calibration(NodeConfig& C, Stream* S, FrameCounter& F, uint8_t* E) : serial(S), Cfg(C), Fcnt(F), encryption_key(E)
{
    authenticated = false;
    if (encryption_key == NULL)
    {
        authenticated = true; 
    }
}

/*
Calibration::Calibration(NodeConfig& C, Stream* S) : serial(S), Cfg(C)
{
    authenticated = true; // don't use encrytion of EEPROM
    encryption_key = NULL;
}
*/


void Calibration::error_message(char* msg)
{
    this->serial->print("unknown command: ");
    this->serial->println(msg);
}



uint16_t Calibration::checksum_crc16(uint8_t *data, uint8_t data_len)
{
    uint16_t crc = 0xFFFF;
    //uint8_t data_len = offsetof(NodeConfig, checksum);
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


uint16_t Calibration::checksum(void)
{
    return checksum_crc16((uint8_t*) &Cfg, offsetof(NodeConfig, checksum));
}


bool Calibration::verify_checksum(void) // perform Check over Cfg structure.
{
    return this->verify_checksum_crc16();
}

bool Calibration::verify_checksum_crc16(void)
{
    uint16_t cs = checksum_crc16((uint8_t*) &Cfg, offsetof(NodeConfig, checksum)) ^ Cfg.checksum;
    if (cs != 0)
        return false;
    return true;
}


bool Calibration::authenticate (char* str2parse)
{
    if (strcmp(str2parse, (char*) encryption_key) && encryption_key != NULL)
    {
        serial->println("Pass not OK");
        return false;
    }
    else
    {
        serial->println("Pass OK");
        return true;
    }
}

/**** parser for calibration commands over serial port                    ***/
/* protocol is fully verbose, data are sent/received as human readable strings

w,<addr>,<value>    write byte <value> to configuration address <addr>
wi,<addr>,<value>   write 16 bit integer to configuration
wf,<addr>,<value>   write float to configuration (4 bytes)

r,<addr>            read byte at configuration address <addr>
ri,<addr>           read 16 bit integer at configuration address <addr>
rf,<addr>           read float at configuration address <addr>

c                   measure ADC and store
cs                  verify checksum and report
a                   list configuration content, byte wise
s                   calculate checksum and update
m                   measure VCC with calibrated values
fe                  measure FEI in RX mode from a reference signal and copy to configuration
x                   exit calibration mode
*/

bool Calibration::parse (char* str2parse)
{

    char *cmd, *ptr;
    int addr;
    bool calibration_done = false;
    uint8_t *pcfg = (uint8_t*) &Cfg;

    cmd = strtok(str2parse, ",");
    ptr = strtok(NULL, ",");
    if (ptr != NULL)
    {
       addr = atoi(ptr);
       if (addr <0 || addr > 1023) addr = -1;
    }
    else
       addr =-1;



    if (addr >=0) // write or read command
    {
        ptr = strtok(NULL, ","); // pointer to value, if any
        if (ptr != NULL) // write command
        {
            if (cmd[0] == 'w')
            {
                if (cmd[1] =='f') // write float
                {
                    float val_f;
                    val_f = atof(ptr);
                    *((float*) (pcfg+addr)) = val_f;
                    serial->print((float)*(pcfg+addr) , DEC);
                }
                else if (cmd[1] =='i') // write int
                {
                    int16_t val_i= atoi(ptr);
                    if ((unsigned int) addr > sizeof(Cfg)-2)
                    {
                        EEPROM.put(addr, val_i);
                        EEPROM.get(addr, val_i);
                        serial->print(val_i, DEC);
                    }
                    else
                    {
                        *((int16_t*) (pcfg+addr)) = val_i; // code crash if addr is external of Cfg
                        serial->print(*((int16_t*) (pcfg+addr)), DEC);
                    }
                }
                else // write byte
                {
                    byte val = atoi(ptr);
                    if ((unsigned int)addr >= sizeof(Cfg))
                    {
                        EEPROM.put(addr, val);
                        serial->print(val, DEC);
                    }
                    else
                    {
                        *((byte*) (pcfg+addr)) = val;
                        serial->print((byte) *(pcfg+addr), DEC);
                    }
                }
                serial->println("");
            }
            else
            {
                error_message(str2parse);
                return false;
            }
        }
        else  // read command
        {
            if(cmd[0] == 'r') //read command
            {
                if (cmd[1]=='f') // read float
                {
                    serial->print(*((float*) (pcfg+addr)), DEC);
                }
                else if (cmd[1]=='i')
                {
                    if ((unsigned int)addr > sizeof(Cfg)-2)
                    {
                        int16_t val_i;
                        EEPROM.get(addr,val_i);
                        serial->print(val_i);
                    }
                    else
                    {
                        serial->print(*((int16_t*) (pcfg+addr)));
                    }
                }
                else // read byte
                {
                    if ((unsigned int)addr >= sizeof(Cfg))
                    {
                        byte val_i;
                        EEPROM.get(addr,val_i);
                        serial->print(val_i);
                    }
                    else
                     serial->print((byte) *(pcfg+addr), DEC);
                }
                serial->println("");
            }
        }
    }

    else // command with no address specifier - neither read nor write
    {
        if (cmd[0] == 'c' && cmd[1] =='s') // cs -verify checksum and report
        {
            serial->print("cs ");
            if (!verify_checksum())
                serial->print("not ");
            serial->println("OK");
        }
        else if (cmd[0] == 'c') // measure ADC and store in EEPROM
        {
            Cfg.AdcCalValue = readVcc();
            serial->print("vcc_adc,");
            serial->print(Cfg.AdcCalValue, DEC);
            serial->println("");
        }
        else if (cmd[0] == 'a') // report EEPROM content
        {
             serial->print("a");
             //for (uint16_t i=0; i< offsetof(NodeConfig, checksum) +2; i++)
             for (uint16_t i=0; i< sizeof(NodeConfig) + 16; i++)
             {
                 serial->print(",");
                 //serial->print((byte) *(pcfg+i), DEC);
                 serial->print((char) *(pcfg+i));
             }
             serial->println("");
        }
        else if (cmd[0] == 's') // update checksum. 'c' is already gone for 'calibrate'
        {
            Cfg.checksum = checksum();
            serial->print(Cfg.checksum, HEX);
            serial->println("");
        }

        else if (cmd[0] == 'm') // measure VCC
        {
            long vref = (long)Cfg.AdcCalValue * Cfg.VccAtCalmV;
            serial->print("vcc_adc,");
            serial->print(vref / readVcc(), DEC);
            serial->println("");
        }
        
        else if (cmd[0] =='r') // reset frame counter
        {
            Fcnt.reset();
            serial->println("TX Frame Counter Reset");
        }
        
        else if (cmd[0] == 'x') // exit calibration mode
        {
            calibration_done = true;
        }
        
        else//
            error_message(str2parse);
    }
    return calibration_done;
}

void Calibration::calibrate()
{
    bool cal_done = false;
    long tstart=millis();;
    unsigned char  led =0;
    uint8_t i=0;
    char inputstr[MAXINPUTSTRINGLENGTH];
    *inputstr=0;

    serial->println("calibration mode.");
    //Cfg.EepromVersionNumber = EEPROMVERSIONNUMBER;
    //Cfg.SoftwareversionNumber = SoftwareVersion;

    //#ifdef USE_CRYSTAL
    //Cfg.UseCrystalRtc = (uint8_t) 1;
    //#else
    //Cfg.UseCrystalRtc = (uint8_t) 0;
    //#endif

    while (!cal_done)
    {
        while (serial->available() > 0)
        {
            char bt = serial->read();
            if (bt == '\r' || bt == '\n')
            {
                if (i < MAXINPUTSTRINGLENGTH)
                {
                    inputstr[i] = 0;
                    //serial->print("received string: "); serial->println(inputstr); // for debug purposes only.
                    (authenticated) ? cal_done = this->parse(inputstr) : authenticated = this->authenticate(inputstr);
                    //serial->print("1 cal done? "); serial->println(cal_done); // for debug only.
                    i=0;
                }
            }
            else
            {
                inputstr[i] = bt;
                i++;
                if (i >= MAXINPUTSTRINGLENGTH) // overflow
                {
                    inputstr[MAXINPUTSTRINGLENGTH -1] = 0;
                    (authenticated) ? cal_done = this->parse(inputstr) : authenticated = this->authenticate(inputstr);
                    i=0;
                }
            }


            if (cal_done) break;
        }

        if (((millis() - tstart) > 125))
        {
            activityLed(led); // LED on
            led ^= 1;
            tstart = millis();
        }
    }
    /*
    if (encryption_key != NULL)
    {
        xxtea Xxtea((uint8_t*) encryption_key, (uint8_t*) &Cfg);
        Xxtea.crypter(encrypt, sizeof(Cfg));
        EEPROM.put(0, Cfg);
        // got 2 choices: encrypt and de-crypt like here, or make a copy of Cfg and encrypt it for the EEPROM.
        Xxtea.crypter(decrypt, sizeof(Cfg));
    }
    else
    {
        EEPROM.put(0, Cfg); // no encryption
    }
    */
    this->put_config_to_eeprom();
    activityLed(0);
}

void Calibration::put_config_to_eeprom(void)
{
    if (this->encryption_key != NULL)
    {
        // make a copy of Cfg and encrypt it for the EEPROM.
        NodeConfig Cfg_encrypted = this->Cfg;
        
        xxtea Xxtea((uint8_t*) this->encryption_key, (uint8_t*) &Cfg_encrypted);
        Xxtea.crypter(encrypt, sizeof(this->Cfg));
        EEPROM.put(0, Cfg_encrypted);
    }
    else
    {
        EEPROM.put(0, this->Cfg); // no encryption
    }
}


void Calibration::get_config_from_eeprom(void)
{
    EEPROM.get(0, this->Cfg);
    if (this->encryption_key != NULL)
    {
        xxtea Xxtea((uint8_t*) this->encryption_key, (uint8_t*) &Cfg);
        Xxtea.crypter(decrypt, sizeof(this->Cfg)); //Decrypt it
    }
}


void Calibration::configure()
{
    /*
    EEPROM.get(0, Cfg);
    if (encryption_key != NULL)
    {
        xxtea Xxtea((uint8_t*) encryption_key, (uint8_t*) &Cfg);
        Xxtea.crypter(decrypt, sizeof(Cfg)); //Decrypt it
    }
    */

    this->get_config_from_eeprom();
    
    if(!this->verify_checksum()) // while loop instead? so we would force a valid checksum?
    {
        // checksum not ok
        serial->println("checksum not ok");
        // initialize the Cfg structure with default values.
        {
            NodeConfig nCfg;
            Cfg = nCfg;
        }
        this->calibrate(); // checksum failed, so we have to calibrate it.
        serial->println("Calibration finished");
    }
    else
    {
        // checksum ok, so it is likely we have valid data in the EEPROM
        serial->println("CAL?"); // ask
        int start = millis();
        bool done = false;
        while ((millis() - start) < 250) // wait for an echo
        {
            if (serial->available() > 0)
            {
                char bt = serial->read();
                if (bt =='y')// there was an answer from a calibration engine.
                {
                    done = true;
                    this->calibrate();
                    serial->println("Calibration finished");
                    break;
                }
                //else
                    //serial->println("Garbage!");
            }
        }
        if (!done)
          serial->println("start app");
    }
    if (!this->verify_checksum())
        serial->println("EEPROM checksum failed.");
}


void Calibration::activityLed (unsigned char state, unsigned int time)
{
  if (Cfg.LedPin>0) {
    pinMode(Cfg.LedPin, OUTPUT);
    if (time == 0) {
      digitalWrite(Cfg.LedPin, state);
    } else {
      digitalWrite(Cfg.LedPin, state);
      delay(time);
      digitalWrite(Cfg.LedPin, !state);
    }
  }
}