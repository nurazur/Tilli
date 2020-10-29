![](https://github.com/nurazur/TiLLi/blob/master/Tilli-logoV0.0.jpg)
# Tilli
***Ti***ny ***L***oRa ***Li***nk<br>
a battery powered wireless sensor or wireless actor using LoRa technology.
Target of the project is the development of small size , cost effective battery powered LoRa nodes for IoT.

### Project
Tilli is based on [TiNo](https://github.com/nurazur/TiNo), a wireless sensor in a matchbox. Its footprint fits a RFM95 module using the Semtech SX1276 LoRa transceiver. Together with the ATmega328P AVR processor the hardware can turn into a low cost yet powerful LoRa node. <br>
The following sensors are supported:
- a motion detector
- a brightness detector, using a photodiode
-up to four configurable pin change interrupts such as tactile switches, reed contacts, vibration sensors or tilt sensors.
- DS18B20 temperature sensor
- SHT20/SHT21/SHT25 and HTU21D temperature and humidity sensors
- BME280 pressure, humidity and temperature sensor

### Firmware
TiLLi Firmware uses the LMIC "Lora Mac in C" Stack available at [github](https://github.com/mcci-catena/arduino-lorawan)


#### required changes to the mcci-catena LMIC stack
A few changes have to be made to make Tilly work correctly:

1. replace the file `lmic_project_config.h` in your *MCCI_LoRaWAN_LMIC* library directory. this removes the support of LoRa Class B, not needed for this project and just occupying  flash and RAM space.
2. Change to your region. Default is `CFG_eu868`.
3. replace the file `config.h` in the */src/lmic* directory. This is needed to allow stable
 Downlink messages, because Tilly's processor clock is - by design - less accurate than the developpers of LMIC assumed.
4. replace the file `radio.c` in the */src/lmic* directory with `radio.cpp`. This is needed to measure VCC during the TX slot. Voltage will drop depending on the battery's internal resistance. Experience shows that batteries can maintain their nominal voltage but internal resistance increases with age and discharge considerably. it is critical to maintain voltage above 1.8V during the TX burst, which is the moment of highest current drain.

#### library dependencies
- [LowPower](https://github.com/rocketscream/Low-Power) under creative commons attribution-sharealike 3.0 unported license.
- [PinChangeInterrupt](https://github.com/NicoHood/PinChangeInterrupt) under open source license, exact type is not specified but looks like GNU
- [SoftwareWire](https://github.com/Testato/SoftwareWire) under GNU GPL 3.0, used in case devices that communicate through I2C bus are used.
- [HTU21D](https://github.com/enjoyneering/HTU21D) under GNU GPL, modified to work with [SoftwareWire](https://github.com/Testato/SoftwareWire). Please pull this library from my repository and copy into your library directory.
-[BME280](https://github.com/finitespace/BME280) under GNU GPL license, modified to work with [SoftwareWire](https://github.com/Testato/SoftwareWire). Please pull this library from my repository and copy into your library directory.
-[OneWire](https://github.com/PaulStoffregen/OneWire), license unknown. Used for OneWire devices like the DS18B20 temperature sensor.
-[DallasTemperature](https://github.com/milesburton/Arduino-Temperature-Control-Library)under GNU LGPL license. Used for DS18B20 temperature sensor.

#### Limitations
- ABP (activation by personalization) only
- LoRa Keys need to be copied into source code.
- individual node configuration possible, but by LoRa downlink only.
