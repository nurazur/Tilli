# TiLLi
![](https://github.com/nurazur/TiLLi/blob/master/Tilli-logoV0.0.jpg)![](https://github.com/nurazur/TiNo/blob/master/matchbox.jpg)

# Einführung
"**Ti**ny **L**ong range **Li**nk" : Batteriebetriebener LoRa Funksensor.

- LoRa LMIC (Lora Mac in C) Firmware - LoRa Class A 1.03 kompatibel
- minimale Kosten (Stückkosten unter 7 EUR)
- minimale Grösse (Streichholzschachtel)
- minimaler Stromverbrauch
- maximale Batterielebensdauer (1 Jahr oder mehr)
- maximale Reichweite bis zu 10km
- super einfach nachzubauen
- Plug&Play Firmware

Folgende Sensoren sind implementiert:
- Bewgungssensor (PIR)
- Helligkeitssensor
- bis zu 4 konfigurierbare Pin-change Interrupts, etwa für Taster, Reed Kontakte, Vibration oder Schieflage ("tilt") Sensoren.
- DS18B20 Temperatur Sensor
- SHT20/SHT21/SHT25 und HTU21D Temperatur und Luftfeuchte Sensoren
- BME280 Luftdruck, Feuchtigkeit und Temperatur Sensor

im Prinzip können alle Arten von Sensoren verwendet werden. ob Temperatur, Luftfeuchtigkeit, Luftdruck, Höhenmesser, Lichtintensität, UV Index, Anwesenheitssensoren, Magnetschalter, Erschütterungs-Sensoren, usw.  Voraussetzung ist dass der Sensor bis mindestens 2.2V Betriebsspannung spezifiziert ist. Sonst kann die Batterieladung nicht voll ausgenutzt werden.

Die Leiterplatten passen zu im Handel erhältlichen PVC Gehäusen, welche in etwa die Grösse einer Streichholzschachtel haben. Die verwendeten Komponenten sind am Markt eingeführt, jederzeit erhältlich und
dadurch kostengünstig zu beschaffen.
# Features

## Allgemein
- kompakte Bauform (Streichholzschachtel).
- Leiterplatten speziell für das Gehäuse.
- Schaltung konsequent auf mimimalem Stromverbrauch optimiert.
- Konzept der minimalen Kosten. Keine teuren Features die k(aum)einer braucht.
- Bidirektionale Funkverbindung (LoRa Class A, Version 1.03)
- Spannung von ca. 1.8V bis 3.6V
- Betrieb mit CR2032 Zelle bis zu 1 Jahr Lebensdauer
- Leiterplatte passend zu im Handel erhältlichen Gehäusen

## Hardware
  - TiNo-HP PCB mounted with RFM95
  - Tilli V0.2 mounted with RFM95
  - Megabrick available at [Tindie Shop](https://www.tindie.com/products/lps/lorawan-megabrick/)

#### Radio
- HopeRF RFM95 Module
- **bidirektionale** Kommunikation
- ISM Band (Europa: 868MHz, US: 915Mhz)
- LoRa Modulation
- Sendeleistung -4 dBm (typ.) bis 20dBm (max)
- Link Budget bis 140dB
- Empfindlichkeit -125 dBm
- Reichweite t.b.d., ist aber extrem weit (2km sind normal)
- HF Kommunikation verschlüsselt

#### Basisband
- Atmel (Microchip) ATMega328p-au mit 32kByte Flash
- Atmel (Microchip) ATMega644P oder ATMega1284P mit 64 bzw. 128 kByte Flash
- Ruhestrom < 2uA mit externem Quarz
- Ruhestrom ca. 4uA mit internem RC Oszillator
- 1 MHz Takt Sender erlaubt Betriebsspannung bis 1.8V
- I2C für Sensoren
- mindestens 4 weitere GPIO

#### Sensoren
- Bewgungsmelder (PIR)
- Helligkeitssensor mittels Photodiode oder LED
- HTU21D (Luftfeuchte, Temperatur)
- SHT21, SHT20, SHT25 (Luftfeuchte, Temperatur)
- BME280 (Luftdruck, Luftfeuchte, Temperatur)
- DS18B20 (Temperatur(en))
- ~~SHT30, SHT31, SHT35~~
- I2C Bus Basierte Sensoren leicht implementierbar
- 4 digitale GPIOs

## System / Software
- Open Source Software C++
- Software kann einfach individuell angepasst werden
- Programmierung mit Arduino IDE
- ~~Konfiguration der Nodes über serielles Interface (FTDI Adapter)~~
- Konfigurations- und Kalibrierdaten im EEPROM gespeichert.
- ~~EEPROM verschlüsselt~~
- Flashen
  - mit ISP Adapter oder
  - seriell mit FTDI Adapter über Bootloader
- bis zu 4 externe Interrupts (z.B. 4 Tasten) konfigurierbar


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

#### Einschränkungen
- Nur ABP (activation by personalization), OTTA wid noch nicht unterstützt.
- TheThingsNetwork V3 Stack noch nicht getestet. Nur V2 wird unterstützt.
- Der Flash Speicher ist auf Platformen mit ATMega328P sehr begrenzt. Mit Blick auf die Weiterentwicklung des Tilli (TTN V3!) empfehle ich dringend eine Platform mit ATMega644P oder ATMega1284P.

## Dokumentation in Deutsch
~~[deutsche Dkumentation](https://github.com/nurazur/TiNo/blob/master/dokumentation.md)~~
kommt.
