#include <Arduino.h> 
#include <EEPROM.h>
#include "framecounter.h"

FrameCounter::FrameCounter(uint16_t e) : eeprom_addr(e)
{
}


void FrameCounter::begin(void)
{
    if (!is_eeprom_valid())
    {
        this->fill_eeprom_with_zeros();
        this-> validate_eeprom();
        this->frame_counter =0;
    }
    else
    {
        this->read_frame_counter();
    }
}


bool FrameCounter::is_eeprom_valid(void)
{
    return (EEPROM.read(eeprom_addr+32*sizeof(frame_counter))== 0x55);
}


void FrameCounter::validate_eeprom(void)
{
    EEPROM.write(eeprom_addr+32*sizeof(frame_counter), 0x55);
}

// get the NEXT frame counter value; call this to load the NEXT FrameCounter::frame_counter value
uint16_t FrameCounter::read_frame_counter() 
{
    uint16_t value =0;
    for (uint16_t pos= this->eeprom_addr; pos< this->eeprom_addr+32*sizeof(frame_counter); pos +=2)
    {
        uint16_t val_at_pos;
        EEPROM.get(pos, val_at_pos);
        
        if (val_at_pos > value)
        {
            value = val_at_pos;
        }
    }
    this->frame_counter = value;
    return value;
}


void FrameCounter::fill_eeprom_with_zeros()
{
    for (uint16_t i=eeprom_addr; i<eeprom_addr+32*sizeof(frame_counter); i++)
        EEPROM.write(i, 0);
}


//Call this AFTER a packet has been sent with LMIC.SeqnoUp value. Stores the NEXT Frame counter value. 
void FrameCounter::set_frame_counter(uint16_t new_value)
{
    uint16_t pos  = (new_value % 32) * 2 + this->eeprom_addr;
    EEPROM.put(pos, new_value);
}
