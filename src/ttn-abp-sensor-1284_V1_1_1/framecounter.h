/*
USAGE: spread framecounter storage in EEPROM over 32 positions.
Lifetime of EEPROM is specified approx. 100000 writes. 
So lifetime is extended to 3200000 writes, this is 36 years if 10 packets are sent per hour 
on average.


IMPLEMENTATION:
FrameCounter FrmCnt(EEPROMADDRESS);

in setup():
FrmCnt.begin();
LMIC.seqnoUp = FrmCnt.get_frame_counter();

When packet has been sent: FrmCnt.set_frame_counter(LMIC.seqnoUp);
*/
#ifndef FRAMECOUNTER
#define FRAMECOUNTER
class FrameCounter
{
    public:
        FrameCounter(uint16_t eeprom_address); //address in eeprom
        void begin(void);
        
        void set_frame_counter(uint16_t new_value);
        uint16_t get_frame_counter(void){ return this->frame_counter;}
        void reset(void);

    protected:
        uint16_t read_frame_counter(void);
        bool is_eeprom_valid(void);
        void validate_eeprom(void);
        void fill_eeprom_with_zeros(void);
        

        uint16_t eeprom_addr;
        uint16_t frame_counter;
};

#endif