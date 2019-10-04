 # Description
LoRaWan modem running on Adafruit [Feather M0 LoRa](https://www.adafruit.com/product/3178) board based on [Basic MAC](https://github.com/lorabasics/basicmac) library. Adopted from original IBM LoRaWAN modem running on IMST WiMOD module. Find the description of available AT commands in `doc/LMiC-Modem.pdf` document.
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
Resulting `feather_m0_lora_modem_eu868.bin` executable for eu868 band by default.
Various RF bands supported, select prefered using:
```
$ make BAND=band
```
where band of eu868, as923, us915, au915, cn470 is available
# Usage
Adafruit feather M0 LoRa module is shipped with bossa bootloader. Get the [Bossac](https://github.com/shumatech/BOSSA/releases/tag/1.7.0) command line tool (verified with version 1.7.0) and load firmware:
```
$ bossac -p PORT -e -w -v -R feather_m0_lora_modem_xxxxx.bin
```
