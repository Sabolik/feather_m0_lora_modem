# Description
LoRaWan modem running on Adafruit [Feather M0 LoRa](https://www.adafruit.com/product/3178) board based on [Basic MAC](https://github.com/lorabasics/basicmac) library. Adopted from original IBM LoRaWAN modem running on IMST WiMOD module. Find the description of available AT commands in `doc/LMiC-Modem.pdf` document. Various RF bands supported, default (factory reset) band is eu868. Supports Low power mode when running on battery.
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
# Usage
Adafruit feather M0 LoRa module is shipped with bossa bootloader. Get the [Bossac](https://github.com/shumatech/BOSSA/releases/tag/1.7.0) command line tool (verified with version 1.7.0). Enter the bootloader by double-clicking the reset button. The onboard red LED should pulse on and off. Load the firmware:
```
$ bossac -p PORT -e -w -v -R feather_m0_lora_modem.bin
```
