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

#include <Arduino.h>   
#ifndef CALIBRATION
#define CALIBRATION

#define MAXINPUTSTRINGLENGTH 32


class Calibration
{
    public:
      Calibration (NodeConfig& C, Stream* S, FrameCounter& F, uint8_t *encryptionkey = NULL);
      //Calibration (NodeConfig& C, Stream* S, uint8_t *encryptionkey = NULL);
      //Calibration (NodeConfig& C, Stream* S);
      
            
      // calculates the checksum over the configuration data stored in EEprom
      uint16_t checksum(void);
      uint16_t checksum_crc16(uint8_t *data, uint8_t datalen);
      
      // calculates the checksum over configuration data stored in EEprom and 
      // compares to the checksum value stored in EEprom.
      bool verify_checksum(void);
      bool verify_checksum_crc16(void);   
      
      // load configuration from eeprom
      void get_config_from_eeprom(void);
      // put configuration into eeprom 
      void put_config_to_eeprom(void);
      
      // actual calibration routine
      void calibrate(); 
      
      // check EEPROM and enter Calibratiion mode if there is a problem 
      // check if there is a request from a calibration program to enter calibration state. 
      void configure();
      
      void activityLed (unsigned char state, unsigned int time = 0);

    protected:
      void error_message(char* msg);
      
      // parser for strings received over serial port
      bool parse (char* str2parse);
      bool authenticate (char* str2parse);
      
    
    private:
        Stream *serial;
        NodeConfig &Cfg;
        FrameCounter &Fcnt;
        bool authenticated;
        uint8_t* encryption_key;
};
#endif