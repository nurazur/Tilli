# Tilli
***Ti***ny ***L***oRa ***Li***nk
a battery powered wireless sensor or wireless actor using LoRa technology.
Target of the project is the development of small size , cost effective battery powered LoRa nodes for IoT.

TiLLi Firmware uses the LMIC "Lora Mac in C" Stack available at [github](https://github.com/mcci-catena/arduino-lorawan)


#### required changes to the mcci-catena LMIC stack
A few changes have to be made to make Tilly work correctly:

1. replace the file `lmic_project_config.h` in your *MCCI_LoRaWAN_LMIC* library directory. this removes the support of LoRa Class B, not needed for this project and just occupying  memory space.
2. Change to your region. Default is `CFG_eu868`.
3. replace the file `config.h` in the */src/lmic* directory. This is needed to allow stable
 Downlink messages, because Tilly's processor clock is - by design - less accurate than the developpers of LMIC assumed.
4. replace the file `radio.c` in the */src/lmic* directory. This is needed to measure VCC during the TX slot. Voltage will drop depending on the battery's internal resistance. Experience shows that batteries can maintain their nominal voltage but internal resistance increases with age and discharge considerably. it is critical to maintain voltage above 1.8V during the TX burst, which is the moment of highest current drain.

#### library dependencies
[LowPower](https://github.com/rocketscream/Low-Power) under creative commons attribution-sharealike 3.0 unported license.
