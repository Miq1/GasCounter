# GasCounter

### Summary
This is a universal pulse counter sketch:
- running on Arduino Nano
- Modbus server interface
- Modbus address selectable by connecting address GPIOs to GND
- counting the times a GPIO is pulled to GND
- rolling 24h history with 3 minute resolution
- 7 days history
- runtime counters (minutes, days)
- recovers from reboots/power loss with minimal data loss. All data held in EEPROM, but any cell is written twice a day only to reduce flash wear.
- Serial debug switch

### Schematic
![Fritzing schematic](https://github.com/Miq1/GasCounter/blob/master/GasCounterSchematic.png?raw=true)

### Modbus register list
Register Address(es)|Data type|Contents|writable?|Remarks
--- | --- | --- | --- | ---
1+2|``uint32_t``|Pulses since last reset| |Reset by writing a new offset value
3+4|``float``|Counter offset: starting value in base units|yes|If written, will reset all counters, history etc.
5+6|``float``|Step size: one pulse in base units|yes|
7..13|``uint16_t``|Pulse counts of the past 7 days, oldest in #7, yesterday in #13| |Will be ``0xFFFF`` when exceeding 65534
14..493|``uint16_t``|Pulse counts for the 480 3-minute periods in the past 24h. Oldest in #14, most recent interval in #493.| | Will be 0xFE when exceeding 253
494|``uint16_t``|Number of reboots since last init| |Useful to detect power losses
495|``uint16_t``|Minutes run since last init| |Will stop at 65535 minutes (~45 days)
496|``uint16_t``|Days since last init| |
497|``uint16_t``|Pulse count for the current day| |

### Application
I wrote this sketch to track my gas volume meter.
The meter ("Kromschröder/Elster BK-G4") has a small magnet in the rightmost digits roll that can be sensed in a groove underneath it.

I finally found a reed sensor sensitive enough to detect the passing magnet (PIC PMC-1401), after trialling a couple before and some hall sensors as well.
This reed sensor will pull the pulse-counting GPIO to GND whenever the magnet passes it.

The meter is a mechanical one, hence the pulse speed is moderately low - normally the frequency is well below 1Hz.
The interrupt-driven counting mechanism in the sketch would be able to catch much higher frequencies, but then the history counters will quickly overflow.
253 pulses in 180s is the maximum, that is, ca. 1.4Hz = 121,440 pulses a day.
This value again is exceeding the daily counters' capacities of 65535, but the main pulse counter will be correct up to the 121,440.

On the other hand, 121,440 pulses are equivalent to 1214,4 m³ gas consumption, i.e. about 13,000 kWh a day - if I ever will need those, I will have interesting problems anyway...
