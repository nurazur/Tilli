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

Als Sensor ist im Moment ein Bewgungssensor (PIR) und ein Helligkeitssensor implementiert. Dies ist aber nur ein Beispiel. So ziemlich alle Sensoren können verwendet werden, ob Temperatur, Luftfeuchtigkeit, Luftdruck, Höhenmesser, Lichtintensität, UV Index,
Anwesenheitssensoren, Magnetschalter, Erschütterungs-Sensoren, usw also im Prinzip alle Arten von Sensoren. Voraussetzung ist dass der Sensor bis mindestens 2.2V Betriebsspannung spezifiziert ist. Sonst kann die Batterieladung nicht voll ausgenutzt werden.

Die Leiterplatten passen zu im Handel erhältlichen PVC Gehäusen, welche in etwa die Grösse einer Streichholzschachtel haben. Die verwendeten Komponenten sind am Markt eingeführt, jederzeit erhältlich und
dadurch kostengünstig zu beschaffen.

# Warum TiLLi?
- kompakte Bauform (Streichholzschachtel).
- Leiterplatten speziell fuer das Gehäuse.
- Schaltung konsequent auf mimimalem Stromverbrauch optimiert. Batterielaufzeit 5 Jahre oder mehr.
- Konzept der minimalen Kosten. Keine teuren Features die k(aum)einer braucht.
- Bidirektionale Funkverbindung (LoRa Class A)


# Features
## Allgemein
- Spannung von ca. 1.8V bis 3.6V
- Betrieb mit CR2032 Zelle bis zu 1 Jahr Lebensdauer
- Leiterplatte passend zu im Handel erhältlichen Gehäusen


## Radio
- HopeRF RFM95 Module
- bidirektionale Kommunikation
- ISM Band (Europa: 868MHz, US: 915Mhz)
- LoRa Modulation
- Sendeleistung -4 dBm (typ.) bis 20dBm (max)
- Link Budget bis 140dB
- Empfindlichkeit -125 dBm
- Reichweite t.b.d., ist aber extrem weit (2km sind normal)
- HF Kommunikation verschlüsselt

## Basisband
- Atmel (Microchip) ATMega328p-au
- 32kByte Flash
- Ruhestrom < 2uA mit externem Quarz
- Ruhestrom ca. 4uA mit internem RC Oszillator
- 1 MHz Takt Sender erlaubt Betriebsspannung bis 1.8V
- I2C für Sensoren
- mindestens 4 weitere GPIO

## Sensoren
- Bewgungsmelder (PIR)
- Helligkeitssensor mittels Photodiode oder lED
- ~~HTU21D~~
- ~~SHT21, SHT20, SHT25~~
- ~~SHT30, SHT31, SHT35~~
- I2C Bus Basierte Sensoren leicht implementierbar
- 4 digitale GPIOs

## System / Software
- Open Source Software C++
- Software kann einfach individuell angepasst werden
- Programmierung mit Arduino IDE
- ~~Konfiguration der Nodes über serielles Interface (FTDI Adapter)~~
- Konfigurations- und Kalibrierdaten im EEPROM gespeichert.
- ~~EEPROM verschlüsselt~~(kommt)
- Flashen
  - mit ISP Adapter oder
  - seriell mit FTDI Adapter über Bootloader
- bis zu 4 externe Interrupts (z.B. 4 Tasten) konfigurierbar


## Dokumentation in Deutsch
~~[deutsche Dkumentation](https://github.com/nurazur/TiNo/blob/master/dokumentation.md)~~
kommt.
