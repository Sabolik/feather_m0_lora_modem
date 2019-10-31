# Description
LoRaWan modem running on Adafruit [Feather M0 LoRa](https://www.adafruit.com/product/3178) board based on [Basic MAC](https://github.com/lorabasics/basicmac) library. Adopted from original IBM LoRaWAN modem running on IMST WiMOD module. Find the description of available AT commands in `doc/LMiC-Modem.pdf` document. Various RF bands supported, default (factory reset) band is eu868. Supports Low power mode when running on battery (standby current <100uA in total from battery, <5uA MCU+Radio).
# Building
Get repository:
```
$ git clone https://github.com/Sabolik/feather_m0_lora_modem
```
Clone required driver and basic MAC submodules locally:
```
$ git submodule update --init --recursive
```
and build directly from `feather_m0_lora_modem` directory:
```
$ make
```
Resulting `feather_m0_lora_modem.bin`
# Loading
Adafruit feather M0 LoRa module is shipped with bossa bootloader. Get the [Bossac](https://github.com/shumatech/BOSSA/releases/tag/1.7.0) command line tool (verified with version 1.7.0). Enter the bootloader by double-clicking the reset button. The onboard red LED should pulse on and off. Load the firmware:
```
$ bossac -p PORT -e -w -v -R feather_m0_lora_modem.bin
```
# Usage
Open your favourite terminal, e.g. PuTTy, select COM port where `Communication Device Class ASF example` is connected, set 115200 bit/s baudrate. Try to send `AT` command, `OK` reply expected.
## Example 1
get weather data from connected BME280 sensor:
```
ATW?
```
reply expected as described in `doc/LMiC-Modem.pdf`, chapter 4.14.2
## Example 2
create [TTN Device](https://www.thethingsnetwork.org/) using ABP activation method to
establish connection as described in `doc/LMiC-Modem.pdf`, chapter 4.5.1
```
ATS=Networ ID (e.g. 00000013),Device Address (MSB),Network Session Key (MSB),App Session Key (MSB)
```
send weather data from connected BME280 sensor on port e.g. 1, channel e.g. 3
```
ATW0,01,03
```
repeat measurement and data sending every 30 minutes:
```
ATL1,708
```
unplug USB and reset device using on-board button. The device is now running in Low power mode (if battery connected) measuring and sending weather data every 30 minutes.
## Example 3
create [TTN Device](https://www.thethingsnetwork.org/) using OTAA activation method
```
ATJ=Device EUI (MSB),Application EUI (MSB),App Key (MSB)
```
send some data to the server on port e.g. 1
```
ATT0,01,C0FFEE
```
new session is established (kept until factory reset is performed) before data is sent
